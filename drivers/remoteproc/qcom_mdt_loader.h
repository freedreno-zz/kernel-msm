#ifndef __QCOM_MDT_LOADER_H__
#define __QCOM_MDT_LOADER_H__

int qcom_mdt_sanity_check(struct rproc *rproc, const struct firmware *fw);
struct resource_table * qcom_mdt_find_rsc_table(struct rproc *rproc, const struct firmware *fw, int *tablesz);
int qcom_mdt_load(struct rproc *rproc, unsigned pas_id, const struct firmware *fw);

#endif
