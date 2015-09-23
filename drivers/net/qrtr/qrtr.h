#ifndef __QRTR_H_
#define __QRTR_H_

#include <linux/list.h>
#include <linux/skbuff.h>

#define QRTR_PEER_NID_AUTO (-1)

struct qrtr_peer;

typedef int (*qrtr_peer_enqueue_t)(struct qrtr_peer *, struct sk_buff *skb);

struct qrtr_peer {
	void *privdata;
};

static inline void qrtr_peer_set_privdata(struct qrtr_peer *peer, void *pdata)
{
	peer->privdata = pdata;
}

static inline void *qrtr_peer_get_privdata(struct qrtr_peer *peer)
{
	return peer->privdata;
}

struct qrtr_peer *qrtr_peer_create(qrtr_peer_enqueue_t cb, unsigned int nid);
void qrtr_peer_destroy(struct qrtr_peer *peer);

int qrtr_peer_post(struct qrtr_peer *peer, const void *data, size_t len);

#endif
