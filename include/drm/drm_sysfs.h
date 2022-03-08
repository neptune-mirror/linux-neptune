/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DRM_SYSFS_H_
#define _DRM_SYSFS_H_

#define DRM_RESET_EVENT_VRAM_LOST (1 << 0)

struct drm_device;
struct device;
struct drm_connector;
struct drm_property;

/**
 * struct drm_reset_event_info - Information about a GPU reset event
 * @pid: Process that triggered the reset, if any
 * @flags: Extra information around the reset event (e.g. is VRAM lost?)
 */
struct drm_reset_event_info {
	struct pid *pid;
	uint64_t flags;
};

int drm_class_device_register(struct device *dev);
void drm_class_device_unregister(struct device *dev);

void drm_sysfs_hotplug_event(struct drm_device *dev);
void drm_sysfs_reset_event(struct drm_device *dev, struct drm_reset_event_info *reset_info);
void drm_sysfs_connector_hotplug_event(struct drm_connector *connector);
void drm_sysfs_connector_status_event(struct drm_connector *connector,
				      struct drm_property *property);
#endif
