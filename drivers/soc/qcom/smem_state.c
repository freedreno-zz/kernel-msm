#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/spinlock.h>

static LIST_HEAD(smem_states);
static DEFINE_MUTEX(list_lock);

struct qcom_smem_state {
	struct kref refcount;

	struct list_head list;
	bool orphan;

	void *data;
	struct device_node *of_node;

	struct qcom_smem_state_ops ops;
};

int qcom_smem_state_update_bits(struct qcom_smem_state *state, u32 mask, u32 value)
{
	if (state->orphan)
		return -ENODEV;

	if (!state->ops.update_bits)
		return -ENOTSUPP;

	return state->ops.update_bits(state->data, mask, value);
}
EXPORT_SYMBOL_GPL(qcom_smem_state_update_bits);

static struct qcom_smem_state *of_node_to_state(struct device_node *np)
{
	struct qcom_smem_state *state;

	mutex_lock(&list_lock);

	list_for_each_entry(state, &smem_states, list) {
		if (state->of_node == np) {
			kref_get(&state->refcount);
			goto unlock;
		}
	}
	state = ERR_PTR(-EPROBE_DEFER);

unlock:
	mutex_unlock(&list_lock);

	return state;
}

struct qcom_smem_state *qcom_smem_state_get(struct device *dev, const char *con_id, unsigned *bit)
{
	struct qcom_smem_state *state;
	struct of_phandle_args args;
	int index = 0;
	int ret;

	if (con_id) {
		index = of_property_match_string(dev->of_node, "qcom,state-names", con_id);
		if (index < 0) {
			dev_err(dev, "missing qcom,state-names\n");
			return ERR_PTR(index);
		}
	}

	ret = of_parse_phandle_with_args(dev->of_node, "qcom,state", "#qcom,state-cells", index, &args);
	if (ret) {
		dev_err(dev, "failed to parse qcom,state property\n");
		return ERR_PTR(ret);
	}

	if (args.args_count != 1) {
		dev_err(dev, "invalid #qcom,state-cells\n");
		return ERR_PTR(-EINVAL);
	}

	state = of_node_to_state(args.np);
	if (IS_ERR(state))
		goto put;

	*bit = args.args[0];

put:
	of_node_put(args.np);
	return state;
}
EXPORT_SYMBOL_GPL(qcom_smem_state_get);

static void qcom_smem_state_release(struct kref *ref)
{
	struct qcom_smem_state *state = container_of(ref, struct qcom_smem_state, refcount);

	list_del(&state->list);
	kfree(state);
}

void qcom_smem_state_put(struct qcom_smem_state *state)
{
	mutex_lock(&list_lock);
	kref_put(&state->refcount, qcom_smem_state_release);
	mutex_unlock(&list_lock);
}
EXPORT_SYMBOL_GPL(qcom_smem_state_put);

struct qcom_smem_state *qcom_smem_state_register(struct device_node *of_node, const struct qcom_smem_state_ops *ops, void *data)
{
	struct qcom_smem_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return ERR_PTR(-ENOMEM);

	kref_init(&state->refcount);

	state->of_node = of_node;
	state->ops = *ops;
	state->data = data;

	mutex_lock(&list_lock);
	list_add(&state->list, &smem_states);
	mutex_unlock(&list_lock);

	return state;
}
EXPORT_SYMBOL_GPL(qcom_smem_state_register);

void qcom_smem_state_unregister(struct qcom_smem_state *state)
{
	state->orphan = true;
	qcom_smem_state_put(state);
}
EXPORT_SYMBOL_GPL(qcom_smem_state_unregister);
