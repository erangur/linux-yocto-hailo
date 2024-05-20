#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_of.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include <linux/clk.h>

struct driver_device {
	void __iomem *regs;
	struct clk *dsi_sys_clk;
	struct clk *dsi_p_clk;
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
};

DEFINE_DRM_GEM_CMA_FOPS(fops);

static const struct drm_driver driver_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &fops,
	DRM_GEM_CMA_DRIVER_OPS,
	.name = "hailo-drm",
	.desc = "HAILO DRM",
	.date = "20240319",
	.major = 1,
	.minor = 0,
};

static const u32 hailo_supported_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const struct drm_mode_config_funcs hailo_drm_modecfg_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int driver_probe(struct platform_device *pdev)
{
	struct driver_device *priv;
	struct drm_device *drm;
	int ret;
	struct drm_panel *panel;
	struct drm_bridge *bridge;

	priv = devm_drm_dev_alloc(&pdev->dev, &driver_drm_driver,
				  struct driver_device, drm);
	if (IS_ERR(priv)) {
		dev_err(&pdev->dev, "failed to allocate drm device\n");
		return PTR_ERR(priv);
	}
	drm = &priv->drm;

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	ret = drmm_mode_config_init(drm);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize mode config\n");
		return ret;
	}

	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = 8096;
	drm->mode_config.max_height = 8096;
	drm->mode_config.funcs = &hailo_drm_modecfg_funcs;

	ret = drm_vblank_init(drm, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize vblank\n");
		return ret;
	}

	ret = drm_of_find_panel_or_bridge(drm->dev->of_node, 0, 0, &panel,
					  &bridge);
	if (ret || !bridge) {
		dev_err(&pdev->dev, "Didn't find panel or bridge\n");
		return ret;
	}

	if (panel) {
		dev_err(&pdev->dev, "in hailo driver_probe - FIND PANEL\n");
		return -ENODEV;
	}

	priv->dsi_sys_clk = devm_clk_get(&pdev->dev, "dsi_sys_clk");
	if (IS_ERR(priv->dsi_sys_clk))
		return PTR_ERR(priv->dsi_sys_clk);

	priv->dsi_p_clk = devm_clk_get(&pdev->dev, "dsi_p_clk");
	if (IS_ERR(priv->dsi_p_clk))
		return PTR_ERR(priv->dsi_p_clk);

	ret = drm_simple_display_pipe_init(drm, &priv->pipe, NULL,
					   hailo_supported_formats,
					   ARRAY_SIZE(hailo_supported_formats),
					   NULL, NULL);

	if (ret) {
		dev_err(&pdev->dev,
			"drm_simple_display_pipe_init return err\n");
		return ret;
	}

	ret = drm_simple_display_pipe_attach_bridge(&priv->pipe, bridge);
	if (ret) {
		dev_err(&pdev->dev,
			"drm_simple_display_pipe_attach_bridge return err\n");
		return ret;
	}

	drm_mode_config_reset(drm);

	platform_set_drvdata(pdev, drm);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to register drm device\n");
		return ret;
	}

	drm_fbdev_generic_setup(drm, 32);

	return 0;
}

// This function is called before the devm_ resources are released
static int driver_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

// This function is called on kernel restart and shutdown
static void driver_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static int __maybe_unused driver_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused driver_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct of_device_id hailo_dt_ids[] = { { .compatible =
							      "hailo,dsi" },
						    { /* sentinel */ } };
MODULE_DEVICE_TABLE(of, hailo_dt_ids);

static const struct dev_pm_ops driver_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	driver_pm_suspend, driver_pm_resume) };

static struct platform_driver driver_driver = {
 		.driver = {
			.name           = "hailo-dsi",
 			.of_match_table	= hailo_dt_ids,
 			.pm = &driver_pm_ops,
 		},
 		.probe = driver_probe,
 		.remove = driver_remove,
 		.shutdown = driver_shutdown,
 	};
module_platform_driver(driver_driver);