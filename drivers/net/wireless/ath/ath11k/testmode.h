/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "core.h"

#ifdef CONFIG_NL80211_TESTMODE

void ath11k_tm_wmi_event_unsegmented(struct ath11k_base *ab, u32 cmd_id,
				     struct sk_buff *skb);
int ath11k_tm_process_event(struct ath11k_base *ab, u32 cmd_id,
			    const struct wmi_ftm_event_msg *ftm_msg,
			    u16 length);
int ath11k_tm_cmd(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  void *data, int len);

#else

static inline void ath11k_tm_wmi_event_unsegmented(struct ath11k_base *ab,
						   u32 cmd_id,
						   struct sk_buff *skb)
{
}

static inline int ath11k_tm_process_event(struct ath11k_base *ab, u32 cmd_id,
					  const struct wmi_ftm_event_msg *msg,
					  u16 length)
{
	return 0;
}

static inline int ath11k_tm_cmd(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				void *data, int len)
{
	return 0;
}

#endif
