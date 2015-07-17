/*
 * Copyright (c) 2015, Sony Mobile Communications.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/termios.h>	/* For TIOCINQ/OUTQ */
#include <linux/socket.h>
#include <linux/qrtr.h>
#include <linux/kernel.h>

#include <net/tcp_states.h>
#include <net/sock.h>

#include "qrtr.h"

#define QRTR_PROTO_VER 1

/* auto-bind range */
#define QRTR_MIN_EPH_SOCKET 0x4000
#define QRTR_MAX_EPH_SOCKET 0x7fff

enum qrtr_pkt_type {
	QRTR_TYPE_DATA		= 1,
	QRTR_TYPE_HELLO		= 2,
	QRTR_TYPE_BYE		= 3,
	QRTR_TYPE_NEW_SERVER	= 4,
	QRTR_TYPE_DEL_SERVER	= 5,
	QRTR_TYPE_DEL_CLIENT	= 6,
	QRTR_TYPE_RESUME_TX	= 7,
	QRTR_TYPE_EXIT		= 8,
	QRTR_TYPE_PING		= 9,
};

struct qrtr_hdr {
	__le32 version;
	__le32 type;
	__le32 src_node_id;
	__le32 src_port_id;
	__le32 confirm_rx;
	__le32 size;
	__le32 dst_node_id;
	__le32 dst_port_id;
} __packed;

#define QRTR_HDR_SIZE sizeof(struct qrtr_hdr)
#define QRTR_NODE_BCAST ((unsigned int)-1)
#define QRTR_PORT_CTRL ((unsigned int)-2)

struct qrtr_sock {
	/* WARNING: sk has to be the first member */
	struct sock sk;
	struct sockaddr_qrtr us;
	struct sockaddr_qrtr peer;
};

static inline struct qrtr_sock *qrtr_sk(struct sock *sk)
{
	BUILD_BUG_ON(offsetof(struct qrtr_sock, sk) != 0);
	return container_of(sk, struct qrtr_sock, sk);
}

static int qrtr_local_peer_nid;

static RADIX_TREE(qrtr_devs, GFP_KERNEL);
static LIST_HEAD(qrtr_all_devs);
static DEFINE_MUTEX(qrtr_dev_lock);

static DEFINE_IDR(qrtr_ports);
static DEFINE_MUTEX(qrtr_port_lock);

struct qrtr_dev {
	struct mutex call_lock;
	struct qrtr_peer peer;
	qrtr_peer_enqueue_t cb;
	struct kref ref;
	unsigned int nid;

	struct sk_buff_head rx_queue;
	struct work_struct work;
	struct list_head item;
};

static void __qrtr_dev_release(struct kref *kref) /* dev mutex locked on call */
{
	struct qrtr_dev *dev = container_of(kref, struct qrtr_dev, ref);

	if (dev->nid != QRTR_PEER_NID_AUTO)
		radix_tree_delete(&qrtr_devs, dev->nid);

	list_del(&dev->item);
	mutex_unlock(&qrtr_dev_lock);

	skb_queue_purge(&dev->rx_queue);
	kfree(dev);
}

static struct qrtr_dev *qrtr_dev_acquire(struct qrtr_dev *dev)
{
	if (dev)
		kref_get(&dev->ref);
	return dev;
}

static void qrtr_dev_release(struct qrtr_dev *dev)
{
	if (!dev)
		return;
	kref_put_mutex(&dev->ref, __qrtr_dev_release, &qrtr_dev_lock);
}

static int qrtr_dev_enqueue(struct qrtr_dev *dev, struct sk_buff *skb)
{
	int rc = -ENODEV;

	mutex_lock(&dev->call_lock);
	if (dev->cb)
		rc = dev->cb(&dev->peer, skb);
	mutex_unlock(&dev->call_lock);

	return rc;
}

/* callers must release with qrtr_dev_release() */
static struct qrtr_dev *qrtr_dev_lookup(unsigned int nid)
{
	struct qrtr_dev *dev;

	mutex_lock(&qrtr_dev_lock);
	dev = radix_tree_lookup(&qrtr_devs, nid);
	dev = qrtr_dev_acquire(dev);
	mutex_unlock(&qrtr_dev_lock);

	return dev;
}

static void qrtr_dev_assign(struct qrtr_dev *dev, unsigned int nid)
{
	if (dev->nid != QRTR_PEER_NID_AUTO || nid == QRTR_PEER_NID_AUTO)
		return;

	mutex_lock(&qrtr_dev_lock);
	radix_tree_insert(&qrtr_devs, nid, dev);
	dev->nid = nid;
	mutex_unlock(&qrtr_dev_lock);
}

int qrtr_peer_post(struct qrtr_peer *peer, const void *data, size_t len)
{
	struct qrtr_dev *dev = container_of(peer, struct qrtr_dev, peer);
	const struct qrtr_hdr *phdr;
	struct sk_buff *skb;
	unsigned int size;
	unsigned int type;
	unsigned int ver;
	unsigned int dst;

	phdr = data;

	if (len < QRTR_HDR_SIZE)
		return -EINVAL;

	ver = le32_to_cpu(phdr->version);
	size = le32_to_cpu(phdr->size);
	type = le32_to_cpu(phdr->type);
	dst = le32_to_cpu(phdr->dst_port_id);

	if (ver != QRTR_PROTO_VER)
		return -EINVAL;

	if (len != size + QRTR_HDR_SIZE)
		return -EINVAL;

	if (dst != QRTR_PORT_CTRL && type != QRTR_TYPE_DATA)
		return -EINVAL;

	skb = netdev_alloc_skb(NULL, len);
	if (skb == NULL)
		return -ENOMEM;

	skb_reset_transport_header(skb);
	skb_put(skb, len);
	skb_store_bits(skb, 0, data, len);

	skb_queue_tail(&dev->rx_queue, skb);

	schedule_work(&dev->work);

	return 0;
}
EXPORT_SYMBOL_GPL(qrtr_peer_post);

static struct qrtr_sock *qrtr_port_lookup(int port);
static void qrtr_port_put(struct qrtr_sock *ipc);

static struct sk_buff *qrtr_alloc_resume_tx(int from_node, int to_node, int port)
{
	struct qrtr_hdr *hdr;
	struct sk_buff *skb;
	const int pkt_len = 20;
	unsigned char buf[pkt_len];

	skb = alloc_skb(QRTR_HDR_SIZE + pkt_len, GFP_KERNEL);
	if (skb == NULL)
		return NULL;

	memset(buf, 0, sizeof(buf));
	((u32 *)buf)[0] = cpu_to_le32(QRTR_TYPE_RESUME_TX);
	((u32 *)buf)[1] = cpu_to_le32(to_node);
	((u32 *)buf)[2] = cpu_to_le32(port);

	skb_reset_transport_header(skb);
	skb_put(skb, pkt_len + QRTR_HDR_SIZE);

	skb_pull(skb, QRTR_HDR_SIZE);
	skb_store_bits(skb, 0, buf, pkt_len);
	skb_push(skb, QRTR_HDR_SIZE);

	hdr = (struct qrtr_hdr *)skb_transport_header(skb);
	hdr->version = cpu_to_le32(QRTR_PROTO_VER);
	hdr->type = cpu_to_le32(QRTR_TYPE_RESUME_TX);
	hdr->src_node_id = cpu_to_le32(from_node);
	hdr->src_port_id = cpu_to_le32(QRTR_PORT_CTRL);
	hdr->confirm_rx = cpu_to_le32(0);
	hdr->size = cpu_to_le32(pkt_len);
	hdr->dst_node_id = cpu_to_le32(to_node);
	hdr->dst_port_id = cpu_to_le32(QRTR_PORT_CTRL);

	return skb;
}

static void qrtr_dev_rx_work(struct work_struct *work)
{
	struct qrtr_dev *dev = container_of(work, struct qrtr_dev, work);
	const struct qrtr_hdr *phdr;
	struct qrtr_sock *ipc;
	struct sk_buff *skb;
	int confirm_rx;
	u32 src_node;
	u32 src_port;
	u32 dst_port;

	while ((skb = skb_dequeue(&dev->rx_queue)) != NULL) {
		phdr = (const struct qrtr_hdr *)skb_transport_header(skb);
		src_node = le32_to_cpu(phdr->src_node_id);
		src_port = le32_to_cpu(phdr->src_port_id);
		dst_port = le32_to_cpu(phdr->dst_port_id);
		confirm_rx = !!phdr->confirm_rx;

		if (src_node > 0)
			qrtr_dev_assign(dev, src_node);

		ipc = qrtr_port_lookup(dst_port);
		if (ipc == NULL) {
			kfree_skb(skb);
			return;
		}

		if (sock_queue_rcv_skb(&ipc->sk, skb))
			kfree_skb(skb);

		qrtr_port_put(ipc);

		if (!confirm_rx)
			continue;

		skb = qrtr_alloc_resume_tx(dev->nid, src_node, src_port);
		if (skb == NULL)
			break;

		if (qrtr_dev_enqueue(dev, skb)) {
			kfree_skb(skb);
			break;
		}
	}
}

struct qrtr_peer *qrtr_peer_create(qrtr_peer_enqueue_t cb, unsigned int nid)
{
	struct qrtr_dev *dev;

	if (cb == NULL)
		return ERR_PTR(-EINVAL);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL)
		return ERR_PTR(-ENOMEM);

	INIT_WORK(&dev->work, qrtr_dev_rx_work);
	kref_init(&dev->ref);
	mutex_init(&dev->call_lock);
	skb_queue_head_init(&dev->rx_queue);
	dev->nid = QRTR_PEER_NID_AUTO;
	dev->cb = cb;

	qrtr_dev_assign(dev, nid);

	mutex_lock(&qrtr_dev_lock);
	list_add(&dev->item, &qrtr_all_devs);
	mutex_unlock(&qrtr_dev_lock);

	return &dev->peer;
}
EXPORT_SYMBOL_GPL(qrtr_peer_create);

void qrtr_peer_destroy(struct qrtr_peer *peer)
{
	struct qrtr_dev *dev = container_of(peer, struct qrtr_dev, peer);

	mutex_lock(&dev->call_lock);
	dev->cb = NULL;
	mutex_unlock(&dev->call_lock);

	qrtr_dev_release(dev);
}
EXPORT_SYMBOL_GPL(qrtr_peer_destroy);

/* callers must release with qrtr_port_put() */
static struct qrtr_sock *qrtr_port_lookup(int port)
{
	struct qrtr_sock *ipc;

	if (port == QRTR_PORT_CTRL)
		port = 0;

	mutex_lock(&qrtr_port_lock);
	ipc = idr_find(&qrtr_ports, port);
	if (ipc)
		sock_hold(&ipc->sk);
	mutex_unlock(&qrtr_port_lock);

	return ipc;
}

static void qrtr_port_put(struct qrtr_sock *ipc)
{
	sock_put(&ipc->sk);
}

static void qrtr_port_remove(struct qrtr_sock *ipc)
{
	int port = ipc->us.sq_port;

	if (port == QRTR_PORT_CTRL)
		port = 0;

	__sock_put(&ipc->sk);

	mutex_lock(&qrtr_port_lock);
	idr_remove(&qrtr_ports, port);
	mutex_unlock(&qrtr_port_lock);
}

static int qrtr_port_assign(struct qrtr_sock *ipc, int *port)
{
	int rc;

	mutex_lock(&qrtr_port_lock);
	if (!*port) {
		rc = idr_alloc(&qrtr_ports, ipc,
				QRTR_MIN_EPH_SOCKET, QRTR_MAX_EPH_SOCKET + 1,
				GFP_ATOMIC);
		if (rc >= 0)
			*port = rc;
	} else if (*port < QRTR_MIN_EPH_SOCKET && !capable(CAP_NET_ADMIN)) {
		rc = -EACCES;
	} else if (*port == QRTR_PORT_CTRL) {
		rc = idr_alloc(&qrtr_ports, ipc, 0, 1, GFP_ATOMIC);
		if (rc == 0)
			*port = QRTR_PORT_CTRL;
	} else {
		rc = idr_alloc(&qrtr_ports, ipc, *port, *port + 1, GFP_ATOMIC);
		if (rc >= 0)
			*port = rc;
	}
	mutex_unlock(&qrtr_port_lock);

	if (rc == -ENOSPC)
		return -EADDRINUSE;
	else if (rc < 0)
		return rc;

	sock_hold(&ipc->sk);

	return 0;
}

static int __qrtr_bind(struct socket *sock,
		const struct sockaddr_qrtr *addr, int zapped)
{
	struct qrtr_sock *ipc = qrtr_sk(sock->sk);
	struct sock *sk = sock->sk;
	int port;
	int rc;

	/* rebinding ok */
	if (!zapped && addr->sq_port == ipc->us.sq_port)
		return 0;

	port = addr->sq_port;
	rc = qrtr_port_assign(ipc, &port);
	if (rc)
		return rc;

	/* unbind previous, if any */
	if (!zapped)
		qrtr_port_remove(ipc);
	ipc->us.sq_port = port;

	sock_reset_flag(sk, SOCK_ZAPPED);

	return 0;
}

static int qrtr_autobind(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct sockaddr_qrtr addr;

	if (!sock_flag(sk, SOCK_ZAPPED))
		return 0;

	addr.sq_family = AF_QIPCRTR;
	addr.sq_node = 0;
	addr.sq_port = 0;

	return __qrtr_bind(sock, &addr, 1);
}

static int qrtr_bind(struct socket *sock, struct sockaddr *saddr, int len)
{
	DECLARE_SOCKADDR(struct sockaddr_qrtr *, addr, saddr);
	struct sock *sk = sock->sk;
	int rc;

	if (len < sizeof(*addr) || addr->sq_family != AF_QIPCRTR)
		return -EINVAL;

	if (addr->sq_node)
		return -EINVAL;

	lock_sock(sk);
	rc = __qrtr_bind(sock, addr, sock_flag(sk, SOCK_ZAPPED));
	release_sock(sk);

	return rc;
}

static int qrtr_local_enqueue(struct qrtr_dev *dev, struct sk_buff *skb)
{
	const struct qrtr_hdr *phdr;
	struct qrtr_sock *ipc;

	phdr = (const struct qrtr_hdr *)skb_transport_header(skb);

	ipc = qrtr_port_lookup(le32_to_cpu(phdr->dst_port_id));
	if (!ipc || &ipc->sk == skb->sk) /* do not send to self */
		return -ENODEV;

	if (sock_queue_rcv_skb(&ipc->sk, skb)) {
		qrtr_port_put(ipc);
		return -ENOSPC;
	}

	qrtr_port_put(ipc);

	return 0;
}

static int qrtr_bcast_enqueue(struct qrtr_dev *dev, struct sk_buff *skb)
{
	struct sk_buff *skbn;
	int rc;

	mutex_lock(&qrtr_dev_lock);
	list_for_each_entry(dev, &qrtr_all_devs, item) {
		skbn = skb_clone(skb, GFP_KERNEL);
		if (skbn == NULL)
			break;
		skb_set_owner_w(skbn, skb->sk);
		rc = qrtr_dev_enqueue(dev, skbn);
		if (rc)
			kfree_skb(skbn);
	}
	mutex_unlock(&qrtr_dev_lock);

	if (qrtr_local_enqueue(dev, skb))
		kfree_skb(skb);

	return 0;
}

static int qrtr_sendmsg(struct socket *sock, struct msghdr *msg, size_t len)
{
	DECLARE_SOCKADDR(struct sockaddr_qrtr *, addr, msg->msg_name);
	int (* enqueue_fn)(struct qrtr_dev *, struct sk_buff *);
	struct qrtr_sock *ipc = qrtr_sk(sock->sk);
	struct sock *sk = sock->sk;
	struct qrtr_dev *dev;
	struct qrtr_hdr *hdr;
	struct sk_buff *skb;
	ssize_t n;
	int size;
	int rc;

	if (msg->msg_flags & ~(MSG_DONTWAIT))
		return -EINVAL;

	if (len > 65536)
		return -EMSGSIZE;

	lock_sock(sk);

	if (addr) {
		if (msg->msg_namelen < sizeof(*addr)) {
			release_sock(sk);
			return -EINVAL;
		}

		if (addr->sq_family != AF_QIPCRTR) {
			release_sock(sk);
			return -EINVAL;
		}

		rc = qrtr_autobind(sock);
		if (rc) {
			release_sock(sk);
			return rc;
		}
	} else if (sk->sk_state == TCP_ESTABLISHED) {
		addr = &ipc->peer;
	} else {
		release_sock(sk);
		return -ENOTCONN;
	}

	dev = NULL;
	if (addr->sq_node == QRTR_NODE_BCAST) {
		enqueue_fn = qrtr_bcast_enqueue;
	} else if (addr->sq_node == 0 || addr->sq_node == ipc->us.sq_node) {
		enqueue_fn = qrtr_local_enqueue;
	} else {
		enqueue_fn = qrtr_dev_enqueue;
		dev = qrtr_dev_lookup(addr->sq_node);
		if (!dev) {
			release_sock(sk);
			return -ECONNRESET;
		}
	}

	size = len + QRTR_HDR_SIZE;

	skb = sock_alloc_send_skb(sk, size, msg->msg_flags & MSG_DONTWAIT, &rc);
	if (!skb)
		goto out_dev;

	skb_reset_transport_header(skb);
	skb_put(skb, size);
	skb_pull(skb, QRTR_HDR_SIZE);

	hdr = (struct qrtr_hdr *)skb_transport_header(skb);

	hdr->version = cpu_to_le32(QRTR_PROTO_VER);
	if (ipc->us.sq_port == QRTR_PORT_CTRL) {
		if (len < 4) {
			rc = -EINVAL;
			goto out_skb;
		}

		/* control messages already require the type as 'command' */
		n = copy_from_iter(&hdr->type, 4, &msg->msg_iter);
		if (n != 4) {
			rc = -EFAULT;
			goto out_skb;
		}
	} else {
		hdr->type = cpu_to_le32(QRTR_TYPE_DATA);
	}

	hdr->src_node_id = cpu_to_le32(ipc->us.sq_node);
	hdr->src_port_id = cpu_to_le32(ipc->us.sq_port);
	hdr->confirm_rx = cpu_to_le32(0);
	hdr->size = cpu_to_le32(len);
	hdr->dst_node_id = cpu_to_le32(addr->sq_node);
	hdr->dst_port_id = cpu_to_le32(addr->sq_port);

	rc = skb_copy_datagram_from_iter(skb, 0, &msg->msg_iter, len);
	if (rc)
		goto out_skb;

	skb_push(skb, QRTR_HDR_SIZE);
	rc = enqueue_fn(dev, skb);
	if (rc >= 0)
		rc = len;

out_skb:
	if (rc < 0)
		kfree_skb(skb);
out_dev:
	qrtr_dev_release(dev);
	release_sock(sk);

	return rc;
}

static int qrtr_recvmsg(struct socket *sock, struct msghdr *msg, size_t size, int flags)
{
	DECLARE_SOCKADDR(struct sockaddr_qrtr *, addr, msg->msg_name);
	struct qrtr_sock *ipc = qrtr_sk(sock->sk);
	const struct qrtr_hdr *phdr;
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int copied, rc;
	u32 src_node;

	lock_sock(sk);

	if (sock_flag(sk, SOCK_ZAPPED)) {
		release_sock(sk);
		return -EADDRNOTAVAIL;
	}

	skb = skb_recv_datagram(sk, flags & ~MSG_DONTWAIT,
				flags & MSG_DONTWAIT, &rc);
	if (!skb) {
		release_sock(sk);
		return rc;
	}

	skb_pull(skb, QRTR_HDR_SIZE);

	copied = skb->len;
	if (copied > size) {
		copied = size;
		msg->msg_flags |= MSG_TRUNC;
	}

	rc = skb_copy_datagram_msg(skb, 0, msg, copied);
	if (rc < 0) {
		skb_free_datagram(sk, skb);
		release_sock(sk);
		return rc;
	}

	phdr = (const struct qrtr_hdr *)skb_transport_header(skb);
	src_node = le32_to_cpu(phdr->src_node_id);

	if (sk->sk_state == TCP_ESTABLISHED && src_node != ipc->peer.sq_node)
		; // FIXME: drop?

	if (addr) {
		addr->sq_family = AF_QIPCRTR;
		addr->sq_node = src_node;
		addr->sq_port = le32_to_cpu(phdr->src_port_id);
		msg->msg_namelen = sizeof(*addr);
	}

	skb_free_datagram(sk, skb);

	release_sock(sk);

	return copied;
}

static int qrtr_connect(struct socket *sock, struct sockaddr *saddr,
		int len, int flags)
{
	DECLARE_SOCKADDR(struct sockaddr_qrtr *, addr, saddr);
	struct qrtr_sock *ipc = qrtr_sk(sock->sk);
	struct sock *sk = sock->sk;
	int rc;

	if (len < sizeof(*addr) || addr->sq_family != AF_QIPCRTR)
		return -EINVAL;

	lock_sock(sk);

	sk->sk_state = TCP_CLOSE;
	sock->state = SS_UNCONNECTED;

	rc = qrtr_autobind(sock);
	if (rc) {
		release_sock(sk);
		return rc;
	}

	ipc->peer = *addr;
	sock->state = SS_CONNECTED;
	sk->sk_state = TCP_ESTABLISHED;

	release_sock(sk);

	return 0;
}

static int qrtr_getname(struct socket *sock, struct sockaddr *saddr,
		int *len, int peer)
{
	struct qrtr_sock *ipc = qrtr_sk(sock->sk);
	struct sockaddr_qrtr qaddr;
	struct sock *sk = sock->sk;

	*len = sizeof(qaddr);

	lock_sock(sk);
	if (peer) {
		if (sk->sk_state != TCP_ESTABLISHED) {
			release_sock(sk);
			return -ENOTCONN;
		}

		qaddr = ipc->peer;
	} else {
		qaddr = ipc->us;
	}
	release_sock(sk);

	qaddr.sq_family = AF_QIPCRTR;

	memcpy(saddr, &qaddr, sizeof(qaddr));

	return 0;
}

static int qrtr_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	long len = 0;
	int rc = 0;

	lock_sock(sk);

	switch (cmd) {
	case TIOCOUTQ:
		len = sk->sk_sndbuf - sk_wmem_alloc_get(sk);
		if (len < 0)
			len = 0;
		rc = put_user(len, (int __user *)argp);
		break;
	case TIOCINQ:
		skb = skb_peek(&sk->sk_receive_queue);
		if (skb)
			len = skb->len - QRTR_HDR_SIZE;
		rc = put_user(len, (int __user *)argp);
		break;
	case SIOCADDRT:
	case SIOCDELRT:
	case SIOCSIFADDR:
	case SIOCGIFADDR:
		rc = -EINVAL;
		break;
	case SIOCGSTAMP:
		rc = sock_get_timestamp(sk, argp);
		break;
	case SIOCGIFDSTADDR:
	case SIOCSIFDSTADDR:
	case SIOCGIFBRDADDR:
	case SIOCSIFBRDADDR:
	case SIOCGIFNETMASK:
	case SIOCSIFNETMASK:
		rc = -EINVAL;
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	release_sock(sk);

	return rc;
}

static int qrtr_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct qrtr_sock *ipc;

	if (!sk)
		return 0;

	lock_sock(sk);

	ipc = qrtr_sk(sk);
	sk->sk_shutdown = SHUTDOWN_MASK;
	if (!sock_flag(sk, SOCK_DEAD))
		sk->sk_state_change(sk);

	sock_set_flag(sk, SOCK_DEAD);
	sock->sk = NULL;

	if (!sock_flag(sk, SOCK_ZAPPED))
		qrtr_port_remove(ipc);

	skb_queue_purge(&sk->sk_receive_queue);

	release_sock(sk);

	sock_put(sk);

	return 0;
}

static const struct proto_ops qrtr_proto_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_QIPCRTR,
	.bind		= qrtr_bind,
	.connect	= qrtr_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.listen		= sock_no_listen,
	.sendmsg	= qrtr_sendmsg,
	.recvmsg	= qrtr_recvmsg,
	.getname	= qrtr_getname,
	.ioctl		= qrtr_ioctl,
	.poll		= datagram_poll,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= sock_no_setsockopt,
	.getsockopt	= sock_no_getsockopt,
	.release	= qrtr_release,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage,
};

static struct proto qrtr_proto = {
	.name		= "QIPCRTR",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct qrtr_sock),
};

static int qrtr_create(struct net *net, struct socket *sock,
		int protocol, int kern)
{
	struct qrtr_sock *ipc;
	struct sock *sk;

	if (sock->type != SOCK_DGRAM)
		return -EPROTOTYPE;

	sk = sk_alloc(net, AF_QIPCRTR, GFP_KERNEL, &qrtr_proto, kern);
	if (!sk)
		return -ENOMEM;

	sock_set_flag(sk, SOCK_ZAPPED);

	sock_init_data(sock, sk);
	sock->ops = &qrtr_proto_ops;
	ipc = qrtr_sk(sk);

	ipc->us.sq_family = AF_QIPCRTR;
	ipc->us.sq_node = qrtr_local_peer_nid;
	ipc->us.sq_port = 0;

	return 0;
}

static const struct net_proto_family qrtr_family = {
	.owner	= THIS_MODULE,
	.family	= AF_QIPCRTR,
	.create	= qrtr_create,
};

static int qrtr_net_up(void)
{
	int rc;

	/* FIXME: sucky, should probably come from DT */
	qrtr_local_peer_nid = 1;

	rc = proto_register(&qrtr_proto, 1);
	if (rc)
		return rc;

	rc = sock_register(&qrtr_family);
	if (rc)
		goto err_sock;

	return 0;

err_sock:
	proto_unregister(&qrtr_proto);
	return rc;
}

static void qrtr_net_down(void)
{
	sock_unregister(qrtr_family.family);
	proto_unregister(&qrtr_proto);
}

static int __init qrtr_init(void)
{
	return qrtr_net_up();
}
module_init(qrtr_init);

static void __exit qrtr_exit(void)
{
	qrtr_net_down();
}
module_exit(qrtr_exit);

MODULE_DESCRIPTION("Qualcomm ipcrouter driver");
MODULE_LICENSE("GPL v2");
