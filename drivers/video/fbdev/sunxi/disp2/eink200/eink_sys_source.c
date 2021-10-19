/*
 * Allwinner SoCs eink driver.
 *
 * Copyright (C) 2019 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "asm/cacheflush.h"
#include <linux/sunxi-gpio.h>
#include "include/eink_sys_source.h"
#include "include/eink_driver.h"

#define BYTE_ALIGN(x) (((x + (4 * 1024 - 1)) >> 12) << 12)

extern struct eink_driver_info g_eink_drvdata;

s32 get_delt_ms_timer(struct timeval start_timer, struct timeval end_timer)
{
	return ((end_timer.tv_sec - start_timer.tv_sec) * 1000 + (end_timer.tv_usec - start_timer.tv_usec)/1000);
}

s32 get_delt_us_timer(struct timeval start_timer, struct timeval end_timer)
{
	return ((end_timer.tv_sec - start_timer.tv_sec) * 1000000 + (end_timer.tv_usec - start_timer.tv_usec));
}

bool is_upd_win_zero(struct upd_win update_area)
{
	if ((update_area.left == 0) && (update_area.right == 0) && (update_area.top == 0) && (update_area.bottom == 0))
		return true;
	else
		return false;
}

int eink_sys_pin_set_state(char *dev_name, char *name)
{
	char compat[32];
	u32 len = 0;
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	struct pinctrl *pctl = NULL;
	struct pinctrl_state *state = NULL;
	int ret = -1;

	len = sprintf(compat, "allwinner,sunxi-%s", dev_name);
	if (len > 32)
		pr_warn("size of mian_name is out of range\n");

	node = of_find_compatible_node(NULL, NULL, compat);
	if (!node) {
		pr_err("of_find_compatible_node %s fail\n", compat);
		goto exit;
	}

	pdev = of_find_device_by_node(node);
	if (!node) {
		pr_err("of_find_device_by_node for %s fail\n", compat);
		goto exit;
	}

	pctl = pinctrl_get(&pdev->dev);
	if (IS_ERR(pctl)) {
		/*not every eink need pin config*/
		pr_err("%s:pinctrl_get for %s fail\n", __func__, compat);
		ret = PTR_ERR(pctl);
		goto exit;
	}

	state = pinctrl_lookup_state(pctl, name);
	if (IS_ERR(state)) {
		pr_err("%s:pinctrl_lookup_state for %s fail\n", __func__, compat);
		ret = PTR_ERR(state);
		goto exit;
	}

	ret = pinctrl_select_state(pctl, state);
	if (ret < 0) {
		pr_err("%s:pinctrl_select_state(%s) for %s fail\n", __func__, name, compat);
		goto exit;
	}
	ret = 0;
exit:
	return ret;
}
EXPORT_SYMBOL(eink_sys_pin_set_state);

int eink_sys_power_enable(char *name)
{
	int ret = 0;
#ifdef CONFIG_AW_AXP
	struct regulator *regu = NULL;

	regu = regulator_get(NULL, name);
	if (IS_ERR(regu)) {
		pr_err("some error happen, fail to get regulator %s\n", name);
		goto exit;
	}
	/* enalbe regulator */
	ret = regulator_enable(regu);
	if (ret != 0) {
		pr_err("some err happen, fail to enable regulator %s!\n", name);
		goto exit1;
	} else {
		pr_info("suceess to enable regulator %s!\n", name);
	}

exit1:
	/* put regulater, when module exit */
	regulator_put(regu);
exit:
#endif
	return ret;
}
EXPORT_SYMBOL(eink_sys_power_enable);

int eink_sys_power_disable(char *name)
{
	int ret = 0;
#ifdef CONFIG_AW_AXP
	struct regulator *regu = NULL;

	regu = regulator_get(NULL, name);
	if (IS_ERR(regu)) {
		pr_err("some error happen, fail to get regulator %s\n", name);
		goto exit;
	}
	/* disalbe regulator */
	ret = regulator_disable(regu);
	if (ret != 0) {
		pr_err("some err happen, fail to disable regulator %s!\n", name);
		goto exit1;
	} else {
		pr_info("suceess to disable regulator %s!\n", name);
	}

exit1:
	/* put regulater, when module exit */
	regulator_put(regu);
exit:
#endif
	return ret;
}
EXPORT_SYMBOL(eink_sys_power_disable);

s32 eink_panel_pin_cfg(u32 en)
{
	struct eink_manager *eink_mgr = get_eink_manager();
	char dev_name[25];

	if (eink_mgr == NULL) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}
	EINK_INFO_MSG("eink pin config, state %s, %d\n", (en) ? "on" : "off", en);

	/* io-pad */
	if (en == 1) {
		if (!((!strcmp(eink_mgr->eink_pin_power, "")) ||
			(!strcmp(eink_mgr->eink_pin_power, "none"))))
			eink_sys_power_enable(eink_mgr->eink_pin_power);
	}

	sprintf(dev_name, "eink");
	eink_sys_pin_set_state(dev_name, (en == 1) ?
			EINK_PIN_STATE_ACTIVE : EINK_PIN_STATE_SLEEP);

	if (en == 0) {
		if (!((!strcmp(eink_mgr->eink_pin_power, "")) ||
			(!strcmp(eink_mgr->eink_pin_power, "none"))))
			eink_sys_power_disable(eink_mgr->eink_pin_power);
	}

	return 0;
}

int eink_sys_gpio_request(struct eink_gpio_cfg *gpio_list,
			  u32 group_count_max)
{
	int ret = 0;
	struct gpio_config pin_cfg;
	char pin_name[32];
	u32 config;
	char pintype[50] = SUNXI_PINCTRL;

	if (gpio_list == NULL) {
		pr_err("%s: gpio list is null\n", __func__);
		return 0;
	}

	pin_cfg.gpio = gpio_list->gpio;
	pin_cfg.mul_sel = gpio_list->mul_sel;
	pin_cfg.pull = gpio_list->pull;
	pin_cfg.drv_level = gpio_list->drv_level;
	pin_cfg.data = gpio_list->data;
	ret = gpio_request(pin_cfg.gpio, NULL);
	if (ret != 0) {
		pr_err("%s failed, gpio_name=%s, gpio=%d, ret=%d\n", __func__,
		      gpio_list->gpio_name, gpio_list->gpio, ret);
		return 0;
	}

	EINK_DEFAULT_MSG("gpio_name=%s, gpio=%3d, <%d,%d,%d,%d> ret=%d\n",
				 gpio_list->gpio_name, gpio_list->gpio,
				 gpio_list->mul_sel, gpio_list->pull,
				 gpio_list->drv_level, gpio_list->data, ret);

	ret = pin_cfg.gpio;

	if (!IS_AXP_PIN(pin_cfg.gpio)) {
		/*
		 * valid pin of sunxi-pinctrl,
		 * config pin attributes individually.
		 */
		sunxi_gpio_to_name(pin_cfg.gpio, pin_name);
		config =
		    SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, pin_cfg.mul_sel);
		ret = pin_config_set(pintype, pin_name, config);
		if (ret == -EINVAL) {
			/*try r_pio*/
			snprintf(pintype, 50, SUNXI_R_PINCTRL);
			ret = pin_config_set(pintype, pin_name, config);
		}
		if (pin_cfg.pull != GPIO_PULL_DEFAULT) {
			config =
			    SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD,
					      pin_cfg.pull);
			pin_config_set(pintype, pin_name, config);
		}
		if (pin_cfg.drv_level != GPIO_DRVLVL_DEFAULT) {
			config =
			    SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV,
					      pin_cfg.drv_level);
			pin_config_set(pintype, pin_name, config);
		}
		if (pin_cfg.data != GPIO_DATA_DEFAULT) {
			config =
			    SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT,
					      pin_cfg.data);
			pin_config_set(pintype, pin_name, config);
		}
	} else if (IS_AXP_PIN(pin_cfg.gpio)) {
		/* valid pin of axp-pinctrl,
		* config pin attributes individually.
		*/
		sunxi_gpio_to_name(pin_cfg.gpio, pin_name);
		if (pin_cfg.data != GPIO_DATA_DEFAULT) {
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT,
			    pin_cfg.data);
			pin_config_set(AXP_PINCTRL, pin_name, config);
		}
	} else {
		pr_warn("invalid pin [%d] from sys-config\n", pin_cfg.gpio);
	}

	return pin_cfg.gpio;
}
EXPORT_SYMBOL(eink_sys_gpio_request);

int eink_sys_gpio_release(int p_handler)
{
	if (p_handler)
		gpio_free(p_handler);
	else
		pr_warn("%s: hdl is NULL\n", __func__);

	return 0;
}
EXPORT_SYMBOL(eink_sys_gpio_release);

int eink_sys_gpio_set_value(u32 p_handler, u32 value_to_gpio,
			    const char *gpio_name)
{
	if (p_handler)
		__gpio_set_value(p_handler, value_to_gpio);
	else
		pr_warn("EINK_SET_GPIO, hdl is NULL\n");

	return 0;
}

/* type: 0:invalid, 1: int; 2:str, 3: gpio */
int eink_sys_script_get_item(char *main_name, char *sub_name, int value[],
			     int type)
{
	char compat[32];
	u32 len = 0;
	struct device_node *node;
	int ret = 0;
	struct gpio_config config;

	len = sprintf(compat, "allwinner,sunxi-%s", main_name);
	if (len > 32)
		pr_err("size of mian_name is out of range\n");

	node = of_find_compatible_node(NULL, NULL, compat);
	if (!node) {
		pr_err("of_find_compatible_node %s fail\n", compat);
		return ret;
	}

	if (type == 1) {
		if (of_property_read_u32_array(node, sub_name, value, 1))
			pr_info("of_property_read_u32_array %s.%s fail\n",
			      main_name, sub_name);
		else
			ret = type;
	} else if (type == 2) {
		const char *str;

		if (of_property_read_string(node, sub_name, &str))
			pr_info("of_property_read_string %s.%s fail\n", main_name,
			      sub_name);
		else {
			ret = type;
			memcpy((void *)value, str, strlen(str) + 1);
		}
	} else if (type == 3) {
		struct eink_gpio_cfg *gpio_info =
		    (struct eink_gpio_cfg *)value;
		int gpio;

		gpio =
		    of_get_named_gpio_flags(node, sub_name, 0,
					    (enum of_gpio_flags *)&config);
		if (!gpio_is_valid(gpio))
			goto exit;

		gpio_info->gpio = config.gpio;
		gpio_info->mul_sel = config.mul_sel;
		gpio_info->pull = config.pull;
		gpio_info->drv_level = config.drv_level;
		gpio_info->data = config.data;
		memcpy(gpio_info->gpio_name, sub_name, strlen(sub_name) + 1);
		__inf("%s.%s gpio=%d,mul_sel=%d,data:%d\n", main_name, sub_name,
		      gpio_info->gpio, gpio_info->mul_sel, gpio_info->data);
		ret = type;
	}

exit:
	return ret;
}
EXPORT_SYMBOL(eink_sys_script_get_item);

#if defined(CONFIG_ION_SUNXI) && defined(EINK_CACHE_MEM)
static int __eink_ion_alloc_coherent(struct ion_client *client, struct eink_ion_mem *mem)
{
	unsigned int flags = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;
	struct scatterlist *sgl = NULL;
	int ret = -1;

	if (IS_ERR_OR_NULL(client) || (mem == NULL)) {
		pr_err("input param is null\n");
		return -1;
	}

	mem->handle = ion_alloc(client, mem->size, PAGE_SIZE,
		(1<<ION_HEAP_TYPE_CARVEOUT), flags);
	if (IS_ERR_OR_NULL(mem->handle)) {
		pr_err("ion_alloc failed, size=%lu!\n", (unsigned long)mem->size);
		return -2;
	}

	mem->vaddr = ion_map_kernel(client, mem->handle);
	if (IS_ERR_OR_NULL(mem->vaddr)) {
		pr_err("ion_map_kernel failed!!\n");
		goto err_map_kernel;
	}

	#ifndef CONFIG_IOMMU_SUPPORT
	if (ion_phys(client, mem->handle, (ion_phys_addr_t *)&mem->paddr, &mem->size)) {
		pr_err("ion_phys failed!!\n");
		goto err_phys;
	}
	#else
	sgl = mem->handle->buffer->sg_table->sgl;
	ret = dma_map_sg(g_eink_drvdata.dmabuf_dev, sgl, 1, DMA_BIDIRECTIONAL);
	if (ret != 1) {
		pr_err("dma map sg fail, ret=%d\n", ret);
		goto err_phys;
	}
	mem->paddr = sg_dma_address(sgl);
	#endif

	EINK_INFO_MSG("ion malloc size=0x%x, vaddr=0x%08x, paddr=0x%08x\n",
			(unsigned int)mem->size, *((unsigned int *)mem->vaddr), *((unsigned int *)mem->paddr));

	return 0;

err_phys:
	ion_unmap_kernel(client, mem->handle);
err_map_kernel:
	ion_free(client, mem->handle);
	return -ENOMEM;
}

static void __eink_ion_free_coherent(struct ion_client *client, struct eink_ion_mem *mem)
{
	if (IS_ERR_OR_NULL(client) || IS_ERR_OR_NULL(mem->handle) || IS_ERR_OR_NULL(mem->vaddr)) {
		pr_err("input param is null\n");
		return;
	}

	ion_unmap_kernel(client, mem->handle);
	ion_free(client, mem->handle);
	return;
}

static void __eink_ion_dump(void)
{
#ifdef ION_DEBUG_DUMP
	struct eink_ion_mgr *ion_mgr =  &(g_eink_drvdata.ion_mgr);
	struct ion_client *client = NULL;
	struct eink_ion_list_node *ion_node = NULL, *tmp_ion_node = NULL;
	struct eink_ion_mem *mem = NULL;
	unsigned int i = 0;

	if (ion_mgr == NULL) {
		pr_err("eink ion mgr is not initial yet\n");
		return;
	}

	client = ion_mgr->client;

	mutex_lock(&(ion_mgr->mlock));
	list_for_each_entry_safe(ion_node, tmp_ion_node, &ion_mgr->ion_list, node) {
		if (ion_node != NULL) {
			mem = &ion_node->mem;
			EINK_INFO_MSG("ion node %d: vaddr=0x%08x, paddr=0x%08x, size=%d\n",
				i, (unsigned int)mem->vaddr, (unsigned int)mem->paddr, mem->size);
			i++;
		}
	}
	mutex_unlock(&(ion_mgr->mlock));
#endif
	return;
}

static void *eink_ion_malloc(u32 num_bytes, void *phys_addr)
{
	struct eink_ion_mgr *ion_mgr =  &(g_eink_drvdata.ion_mgr);
	struct ion_client *client = NULL;
	struct eink_ion_list_node *ion_node = NULL;
	struct eink_ion_mem *mem = NULL;
	u32 *paddr = NULL;
	int ret = -1;

	if (ion_mgr == NULL) {
		pr_err("eink ion manager has not initial yet\n");
		return NULL;
	}

	ion_node = kmalloc(sizeof(*ion_node), GFP_KERNEL);
	if (ion_node == NULL) {
		pr_err("fail to alloc ion node, size=%u\n", (unsigned int)sizeof(*ion_node));
		return NULL;
	}

	mutex_lock(&(ion_mgr->mlock));
	client = ion_mgr->client;
	mem = &ion_node->mem;
	mem->size = BYTE_ALIGN(num_bytes);
	ret = __eink_ion_alloc_coherent(client, mem);
	if (ret != 0) {
		pr_err("fail to alloc ion, ret=%d\n", ret);
		goto err_hdl;
	}

	paddr = (u32 *)phys_addr;
	*paddr = (u32)mem->paddr;
	list_add_tail(&(ion_node->node), &(ion_mgr->ion_list));

	mutex_unlock(&(ion_mgr->mlock));

	__eink_ion_dump();

	return mem->vaddr;

err_hdl:
	kfree(ion_node);
	mutex_unlock(&(ion_mgr->mlock));

	return NULL;
}

static void eink_ion_free(void *virt_addr, void *phys_addr, u32 num_bytes)
{
	struct eink_ion_mgr *ion_mgr =  &(g_eink_drvdata.ion_mgr);
	struct ion_client *client = NULL;
	struct eink_ion_list_node *ion_node = NULL, *tmp_ion_node = NULL;
	struct eink_ion_mem *mem = NULL;
	bool found = false;

	if (ion_mgr == NULL) {
		pr_err("eink ion manager has not initial yet\n");
		return;
	}

	client = ion_mgr->client;

	mutex_lock(&(ion_mgr->mlock));
	list_for_each_entry_safe(ion_node, tmp_ion_node, &ion_mgr->ion_list, node) {
		if (ion_node != NULL) {
			mem = &ion_node->mem;
			if ((((unsigned long)mem->paddr) == ((unsigned long)phys_addr)) &&
				(((unsigned long)mem->vaddr) == ((unsigned long)virt_addr))) {
				__eink_ion_free_coherent(client, mem);
				__list_del_entry(&(ion_node->node));
				found = true;
				break;
			}
		}
	}
	mutex_unlock(&(ion_mgr->mlock));

	if (false == found) {
		pr_err("vaddr=0x%08x, paddr=0x%08x is not found in ion\n",
				*((unsigned int *)virt_addr), *((unsigned int *)phys_addr));
	}

	return;
}
#else
void *eink_mem_malloc(u32 num_bytes, void *phys_addr)
{
	u32 actual_bytes;
	void *address = NULL;

	EINK_DEFAULT_MSG("input!\n");
	if (num_bytes != 0) {
		actual_bytes = ALIGN(num_bytes, PAGE_SIZE);

		address =
		    dma_alloc_coherent(g_eink_drvdata.dmabuf_dev, actual_bytes,
				       (dma_addr_t *) phys_addr, GFP_KERNEL);
		if (address) {
			EINK_INFO_MSG
			    ("dma_alloc_coherent ok, address=0x%p, size=0x%x\n",
			     (void *)(*(unsigned long *)phys_addr), num_bytes);
			return address;
		}

		pr_err("dma_alloc_coherent fail, size=0x%x\n", num_bytes);
		return NULL;
	}

	pr_warn("%s size is zero\n", __func__);

	return NULL;
}

void eink_mem_free(void *virt_addr, void *phys_addr, u32 num_bytes)
{
	u32 actual_bytes;

	actual_bytes = BYTE_ALIGN(num_bytes);
	if (phys_addr && virt_addr)
		dma_free_coherent(g_eink_drvdata.dmabuf_dev, actual_bytes, virt_addr,
				  (dma_addr_t)phys_addr);
}
#endif

void *eink_malloc(u32 num_bytes, void *phys_addr)
{
#if defined(CONFIG_ION_SUNXI) && defined(EINK_CACHE_MEM)
	return eink_ion_malloc(num_bytes, phys_addr); /* cache */
#else
	return eink_mem_malloc(num_bytes, phys_addr); /* skip cache */
#endif
}
EXPORT_SYMBOL(eink_malloc);

void eink_free(void *virt_addr, void *phys_addr, u32 num_bytes)
{
#if defined(CONFIG_ION_SUNXI) && defined(EINK_CACHE_MEM)
	return eink_ion_free(virt_addr, phys_addr, num_bytes);
#else
	return eink_mem_free(virt_addr, phys_addr, num_bytes);
#endif
}
EXPORT_SYMBOL(eink_free);

void eink_cache_sync(void *startAddr, int size)
{
#if defined(CONFIG_ION_SUNXI) && defined(EINK_CACHE_MEM)
	struct sunxi_cache_range range;

	range.start = (unsigned long)startAddr;
	range.end = (unsigned long)startAddr + size;

#ifdef CONFIG_ARM64
	__dma_flush_range((void *)range.start, range.end - range.start);
#else
	dmac_flush_range((void *)range.start, (void *)range.end);
#endif
	return;
#else
	return;
#endif
}
EXPORT_SYMBOL(eink_cache_sync);

int eink_dma_map_core(int fd, struct dmabuf_item *item)
{
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt, *sgt_bak;
	struct scatterlist *sgl, *sgl_bak;
	s32 sg_count = 0;
	int ret = -1;
	int i;

	if (fd < 0) {
		pr_err("[EINK]dma_buf_id(%d) is invalid\n", fd);
		goto exit;
	}
	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		pr_err("[EINK]dma_buf_get failed, fd=%d\n", fd);
		goto exit;
	}

	attachment = dma_buf_attach(dmabuf, g_eink_drvdata.dmabuf_dev);
	if (IS_ERR(attachment)) {
		pr_err("[EINK]dma_buf_attach failed\n");
		goto err_buf_put;
	}
	sgt = dma_buf_map_attachment(attachment, DMA_FROM_DEVICE);
	if (IS_ERR_OR_NULL(sgt)) {
		pr_err("[EINK]dma_buf_map_attachment failed\n");
		goto err_buf_detach;
	}

	/* create a private sgtable base on the given dmabuf */
	sgt_bak = kmalloc(sizeof(struct sg_table), GFP_KERNEL | __GFP_ZERO);
	if (sgt_bak == NULL) {
		pr_err("[EINK]alloc sgt fail\n");
		goto err_buf_unmap;
	}
	ret = sg_alloc_table(sgt_bak, sgt->nents, GFP_KERNEL);
	if (ret != 0) {
		pr_err("[EINK]alloc sgt fail\n");
		goto err_kfree;
	}
	sgl_bak = sgt_bak->sgl;
	for_each_sg(sgt->sgl, sgl, sgt->nents, i)  {
		sg_set_page(sgl_bak, sg_page(sgl), sgl->length, sgl->offset);
		sgl_bak = sg_next(sgl_bak);
	}

	sg_count = dma_map_sg_attrs(g_eink_drvdata.dmabuf_dev, sgt_bak->sgl,
			      sgt_bak->nents, DMA_FROM_DEVICE,
			      DMA_ATTR_SKIP_CPU_SYNC);

	if (sg_count != 1) {
		pr_err("[EINK]dma_map_sg failed:%d\n", sg_count);
		goto err_sgt_free;
	}

	item->fd = fd;
	item->buf = dmabuf;
	item->sgt = sgt_bak;
	item->attachment = attachment;
	item->dma_addr = sg_dma_address(sgt_bak->sgl);
	ret = 0;

	goto exit;

err_sgt_free:
	sg_free_table(sgt_bak);
err_kfree:
	kfree(sgt_bak);
err_buf_unmap:
	/* unmap attachment sgt, not sgt_bak, because it's not alloc yet! */
	dma_buf_unmap_attachment(attachment, sgt, DMA_FROM_DEVICE);
err_buf_detach:
	dma_buf_detach(dmabuf, attachment);
err_buf_put:
	dma_buf_put(dmabuf);
exit:
	return ret;
}

void eink_dma_unmap_core(struct dmabuf_item *item)
{

	dma_unmap_sg_attrs(g_eink_drvdata.dmabuf_dev, item->sgt->sgl,
			      item->sgt->nents, DMA_FROM_DEVICE,
			      DMA_ATTR_SKIP_CPU_SYNC);
	dma_buf_unmap_attachment(item->attachment, item->sgt, DMA_FROM_DEVICE);
	sg_free_table(item->sgt);
	kfree(item->sgt);
	dma_buf_detach(item->buf, item->attachment);
	dma_buf_put(item->buf);
}

struct dmabuf_item *eink_dma_map(int fd)
{
	struct dmabuf_item *item = NULL;

	EINK_INFO_MSG("Func intput!\n");
	item = kmalloc(sizeof(struct dmabuf_item),
			      GFP_KERNEL | __GFP_ZERO);

	if (item == NULL) {
		pr_err("malloc memory of size %d fail!\n",
		       (unsigned int)sizeof(struct dmabuf_item));
		goto exit;
	}
	if (eink_dma_map_core(fd, item) != 0) {
		kfree(item);
		item = NULL;
	}

exit:
	return item;
}

void eink_dma_unmap(struct dmabuf_item *item)
{
	eink_dma_unmap_core(item);
	kfree(item);
}

void eink_print_register(unsigned long start_addr, unsigned long end_addr)
{
	unsigned long start_addr_align = ALIGN(start_addr, 4);
	unsigned long end_addr_align = ALIGN(end_addr, 4);
	unsigned long addr = 0;
	unsigned char tmp_buf[50] = {0};
	unsigned char buf[256] = {0};

	pr_info("print reg: 0x%08x ~ 0x%08x", (unsigned int)start_addr_align, (unsigned int)end_addr_align);
	for (addr = start_addr_align; addr <= end_addr_align; addr += 4) {
		if (0 == (addr & 0xf)) {
			memset(tmp_buf, 0, sizeof(tmp_buf));
			memset(buf, 0, sizeof(buf));
			snprintf(tmp_buf, sizeof(tmp_buf), "0x%08x: ", (unsigned int)addr);
			strncat(buf, tmp_buf, sizeof(tmp_buf));
		}

		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "0x%08x ", readl((void __iomem *)(addr | 0xf0000000)));
		strncat(buf, tmp_buf, sizeof(tmp_buf));
		if (0 == ((addr + 4) & 0xf)) {
			pr_info("%s\n", buf);
			msleep(5);
		}
	}
}

#ifdef PIPELINE_DEBUG
void print_free_pipe_list(struct pipe_manager *mgr)
{
	struct pipe_info_node *pipe, *tpipe;
	unsigned long flags = 0;

	spin_lock_irqsave(&mgr->list_lock, flags);
	if (list_empty(&mgr->pipe_free_list)) {
		EINK_INFO_MSG("FREE_LIST is empty\n");
		spin_unlock_irqrestore(&mgr->list_lock, flags);
		return;
	}

	list_for_each_entry_safe(pipe, tpipe, &mgr->pipe_free_list, node) {
		if (pipe->pipe_id < 10) {
			EINK_INFO_MSG("FREE_LIST: pipe %02d is free\n", pipe->pipe_id);
		}
	}
	spin_unlock_irqrestore(&mgr->list_lock, flags);
}

void print_used_pipe_list(struct pipe_manager *mgr)
{
	struct pipe_info_node *pipe, *tpipe;
	unsigned long flags = 0;

	spin_lock_irqsave(&mgr->list_lock, flags);
	if (list_empty(&mgr->pipe_used_list)) {
		EINK_INFO_MSG("USED_LIST is empty\n");
		spin_unlock_irqrestore(&mgr->list_lock, flags);
		return;
	}

	list_for_each_entry_safe(pipe, tpipe, &mgr->pipe_used_list, node) {
		if (pipe->pipe_id < 10) {
			EINK_INFO_MSG("USED_LIST: pipe %02d is used\n", pipe->pipe_id);
		}
	}
	spin_unlock_irqrestore(&mgr->list_lock, flags);
}
#endif

#ifdef BUFFER_LIST_DEBUG
void print_coll_img_list(struct buf_manager *mgr)
{
	struct img_node *curnode = NULL, *tmpnode = NULL;

	mutex_lock(&mgr->mlock);
	if (list_empty(&mgr->img_collision_list)) {
		EINK_INFO_MSG("COLL_LIST is empty\n");
		mutex_unlock(&mgr->mlock);
		return;
	}

	list_for_each_entry_safe(curnode, tmpnode, &mgr->img_collision_list, node) {
		EINK_INFO_MSG("COLL_LIST: img [buf_id, order, state] [%d, %d, %d] is collision\n",
						curnode->buf_id, curnode->upd_order, curnode->img->state);
	}
	mutex_unlock(&mgr->mlock);
}

void print_used_img_list(struct buf_manager *mgr)
{
	struct img_node *curnode = NULL, *tmpnode = NULL;

	mutex_lock(&mgr->mlock);
	if (list_empty(&mgr->img_used_list)) {
		EINK_INFO_MSG("USED_LIST is empty\n");
		mutex_unlock(&mgr->mlock);
		return;
	}

	list_for_each_entry_safe(curnode, tmpnode, &mgr->img_used_list, node) {
		EINK_INFO_MSG("IMG_USED_LIST: img [buf_id, order, state] [%d, %d, %d] is used\n",
					curnode->buf_id, curnode->upd_order, curnode->img->state);
	}

	mutex_unlock(&mgr->mlock);
}

void print_free_img_list(struct buf_manager *mgr)
{
	struct img_node *curnode = NULL, *tmpnode = NULL;

	mutex_lock(&mgr->mlock);
	if (list_empty(&mgr->img_free_list)) {
		EINK_INFO_MSG("IMG_FREE_LIST is empty\n");
		mutex_unlock(&mgr->mlock);
		return;
	}

	list_for_each_entry_safe(curnode, tmpnode, &mgr->img_free_list, node) {
		EINK_INFO_MSG("FREE_LIST: img [buf_id, order, state] [%d, %d, %d] is free\n",
						curnode->buf_id, curnode->upd_order, curnode->img->state);
	}

	mutex_unlock(&mgr->mlock);
}
#endif

int save_as_bin_file(__u8 *buf, char *file_name, __u32 length, loff_t pos)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	ssize_t ret = 0;

	if ((buf == NULL) || (file_name == NULL)) {
		pr_err("%s: buf or file_name is null\n", __func__);
		return -1;
	}

	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s: fail to open file(%s), ret=%d\n",
					__func__, file_name, *((u32 *)fp));
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_write(fp, buf, length, &pos);
	EINK_INFO_MSG("%s: save %s done, len=%u, pos=%lld, ret=%d\n",
					__func__, file_name, length, pos, (unsigned int)ret);

	set_fs(old_fs);
	filp_close(fp, NULL);

	return ret;

}

//#ifdef SAVE_DE_WB_BUF
s32 save_gray_image_as_bmp(u8 *buf, char *file_name, u32 scn_w, u32 scn_h)
{
	int ret = -1;
	ES_FILE *fd = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	BMP_FILE_HEADER st_file_header;
	BMP_INFO_HEADER st_info_header;
	char *src_buf = buf;
	char *path = file_name;
	BMP_INFO dest_info;
	u32 bit_count = 0;
	ST_RGB *dest_buf = NULL;
	ST_ARGB color_table[256];
	u32 i  = 0;

	if ((path == NULL) || (src_buf == NULL)) {
		pr_err("%s: input param is null\n", __func__);
		return -1;
	}

	fd = filp_open(path, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fd)) {
		pr_err("%s: create bmp file(%s) error\n", __func__, path);
		return -2;
	}

	bit_count = 8;
	memset(&dest_info, 0, sizeof(BMP_INFO));
	dest_info.width = scn_w;
	dest_info.height = scn_h;
	dest_info.bit_count = 8;
	dest_info.color_tbl_size = 0;
	dest_info.image_size = (dest_info.width * dest_info.height * dest_info.bit_count)/8;

	st_file_header.bfType[0] = 'B';
	st_file_header.bfType[1] = 'M';
	st_file_header.bfOffBits = BMP_IMAGE_DATA_OFFSET;
	st_file_header.bfSize = st_file_header.bfOffBits + dest_info.image_size;
	st_file_header.bfReserved1 = 0;
	st_file_header.bfReserved2 = 0;

	st_info_header.biSize = sizeof(BMP_INFO_HEADER);
	st_info_header.biWidth = dest_info.width;
	st_info_header.biHeight = dest_info.height;
	st_info_header.biPlanes = 1;
	st_info_header.biBitCount = dest_info.bit_count;
	st_info_header.biCompress = 0;
	st_info_header.biSizeImage = dest_info.image_size;
	st_info_header.biXPelsPerMeter = 0;
	st_info_header.biYPelsPerMeter = 0;
	st_info_header.biClrUsed = 0;
	st_info_header.biClrImportant = 0;

	for (i = 0; i < 256; i++) {
		color_table[i].reserved = 0;
		color_table[i].r = i;
		color_table[i].g = i;
		color_table[i].b = i;
	}

	dest_buf = (ST_RGB *)src_buf;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_write(fd, (u8 *)(&st_file_header), BMP_FILE_HEADER_SIZE, &pos);
	EINK_INFO_MSG("save file header, len=%u, pos=%d, ret=%d\n", (unsigned int)BMP_FILE_HEADER_SIZE, (int)pos, ret);

	ret = vfs_write(fd, (u8 *)(&st_info_header), BMP_INFO_HEADER_SIZE, &pos);
	EINK_INFO_MSG("save file header, len=%u, pos=%d, ret=%d\n", (unsigned int)BMP_INFO_HEADER_SIZE, (int)pos, ret);

	ret = vfs_write(fd, (u8 *)(color_table), BMP_COLOR_SIZE, &pos);
	EINK_INFO_MSG("save file header, len=%u, pos=%d, ret=%d\n", (unsigned int)BMP_COLOR_SIZE, (int)pos, ret);

	ret = vfs_write(fd, (u8 *)(dest_buf), dest_info.image_size, &pos);
	EINK_INFO_MSG("save file header, len=%u, pos=%d, ret=%d\n", dest_info.image_size, (int)pos, ret);

	set_fs(old_fs);

	ret = 0;

	if (!IS_ERR(fd)) {
		filp_close(fd, NULL);
	}

	return ret;
}

void save_upd_rmi_buffer(u32 order, u8 *buf, u32 len)
{
	char file_name[256] = {0};

	if (buf == NULL) {
		pr_err("%s:input param is null\n", __func__);
		return;
	}

	memset(file_name, 0, sizeof(file_name));
	snprintf(file_name, sizeof(file_name), "/tmp/rmi_buf%d.bin", order);
	save_as_bin_file(buf, file_name, len, 0);
}

void eink_put_gray_to_mem(u32 order, char *buf, u32 width, u32 height)
{
	char file_name[256] = {0};

	if (buf == NULL) {
		pr_err("input param is null\n");
		return;
	}

	memset(file_name, 0, sizeof(file_name));
	snprintf(file_name, sizeof(file_name), "/tmp/eink_image%d.bmp", order);
	save_gray_image_as_bmp((u8 *)buf, file_name, width, height);
}
//#endif

#if (defined DE_WB_DEBUG) || (defined WAVEDATA_DEBUG)
int eink_get_gray_from_mem(__u8 *buf, char *file_name, __u32 length, loff_t pos)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	__s32 read_len = 0;
	ssize_t ret = 0;

	if ((buf == NULL) || (file_name == NULL)) {
		pr_err("%s: buf or file_name is null\n", __func__);
		return -1;
	}

	EINK_INFO_MSG("\n");
	fp = filp_open(file_name, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: fail to open file(%s), ret=0x%p\n",
				__func__, file_name, fp);
		return -1;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	read_len = vfs_read(fp, buf, length, &pos);
	if (read_len != length) {
		pr_err("maybe miss some data(read=%d byte, file=%d byte)\n",
				read_len, length);
		ret = -EAGAIN;
	}
	set_fs(fs);
	filp_close(fp, NULL);

	return ret;
}
#endif

#ifdef WAVEDATA_DEBUG
void save_waveform_to_mem(u32 order, u8 *buf, u32 frames, u32 bit_num)
{
	char file_name[256] = {0};
	u32 len = 0, per_size = 0;

	if (buf == NULL) {
		pr_err("%s:input param is null\n", __func__);
		return;
	}
	if (bit_num == 5)
		per_size = 1024;
	else
		per_size = 256;

	len = frames * per_size;
	memset(file_name, 0, sizeof(file_name));
	snprintf(file_name, sizeof(file_name), "/tmp/waveform%d.bin", order);
	save_as_bin_file(buf, file_name, len, 0);
}

void save_rearray_waveform_to_mem(u8 *buf, u32 len)
{
	char file_name[256] = {0};

	if (buf == NULL) {
		pr_err("%s:input param is null\n", __func__);
		return;
	}
	pr_info("%s:len is %d\n", __func__, len);

	memset(file_name, 0, sizeof(file_name));
	snprintf(file_name, sizeof(file_name), "/tmp/waveform_rearray.bin");
	save_as_bin_file(buf, file_name, len, 0);
}
#endif

#ifdef OFFLINE_SINGLE_MODE
void save_one_wavedata_buffer(u8 *buf, bool is_edma)
{
	struct wavedata_queue *queue = NULL;
	/* u8 *buf = NULL; */
	char file_name[256] = {0};
	static u32 dec_id = 0, edma_id;
	u32 data_len = 0;
	u32 vsync = 0, hsync = 0;
	struct eink_manager *mgr = get_eink_manager();
	struct pipe_manager *pipe_mgr = get_pipeline_manager();

	if (pipe_mgr == NULL) {
		pr_err("%s;pipe mgr is NULL\n", __func__);
		return;
	}

	hsync = mgr->timing_info.lbl + mgr->timing_info.lsl + mgr->timing_info.ldl + mgr->timing_info.lel;
	vsync = mgr->timing_info.fbl + mgr->timing_info.fsl + mgr->timing_info.fdl + mgr->timing_info.fel;

	if (mgr->panel_info.data_len == 8) {
		data_len = hsync * vsync;
	} else {
		data_len = hsync * vsync * 2;
	}

	queue = &pipe_mgr->wavedata_ring_buffer;
	memset(file_name, 0, sizeof(file_name));
	if (is_edma) {
		snprintf(file_name, sizeof(file_name), "/tmp/edma_wf_data%d.bin", edma_id);
		/* buf = (void*)queue->wavedata_vaddr[0]; */
		edma_id++;
	} else {
		snprintf(file_name, sizeof(file_name), "/tmp/dc_wf_data%d.bin", dec_id);
		/* buf = (void*)queue->wavedata_vaddr[0]; */
		dec_id++;
	}

	save_as_bin_file(buf, file_name, data_len, 0);
}

#if 0
static void dump_wavedata_buffer(struct wavedata_queue *queue)
{
	int i = 0;
	unsigned long flags = 0;
	unsigned int head = 0, tail = 0, tmp_tail = 0, tmp_head = 0;
	enum wv_buffer_state buffer_state[WAVE_DATA_BUF_NUM];

	spin_lock_irqsave(&queue->slock, flags);
	head = queue->head;
	tail = queue->tail;
	tmp_tail = queue->tmp_tail;
	tmp_head = queue->tmp_head;

	for (i = 0; i < WAVE_DATA_BUF_NUM; i++)
		buffer_state[i] = queue->buffer_state[i];
	spin_unlock_irqrestore(&queue->slock, flags);

	EINK_INFO_MSG("head=%d, tail=%d, tmp_head=%d, tmp_tail=%d\n", head, tail, tmp_head, tmp_tail);
	for (i = 0; i < WAVE_DATA_BUF_NUM; i++) {
		if (buffer_state[i] != WV_INIT_STATE) {
			printk("miss buffer %d, state=%d\n", i, buffer_state[i]);
		}
	}
}
#endif

void save_all_wavedata_buffer(struct eink_manager *mgr, u32 all_total_frames)
{
	struct wavedata_queue *queue = NULL;
	u8 *buf = NULL;
	char file_name[256] = {0};
	u32 id = 0;
	u32 data_len = 0;
	u32 vsync = 0, hsync = 0;
	struct pipe_manager *pipe_mgr = get_pipeline_manager();

	if (pipe_mgr == NULL) {
		pr_err("%s;pipe mgr is NULL\n", __func__);
		return;
	}

	if (all_total_frames > WAVE_DATA_BUF_NUM) {
		pr_err("too many frames(%d), not to save\n ", all_total_frames);
		return;
	}

	hsync = mgr->timing_info.lbl + mgr->timing_info.lsl + mgr->timing_info.ldl + mgr->timing_info.lel;
	vsync = mgr->timing_info.fbl + mgr->timing_info.fsl + mgr->timing_info.fdl + mgr->timing_info.fel;

	if (mgr->panel_info.data_len == 8) {
		data_len = hsync * vsync;
	} else {
		data_len = hsync * vsync * 2;
	}

	EINK_INFO_MSG("save wavedata, total_frames=%d\n", all_total_frames);
	queue = &pipe_mgr->wavedata_ring_buffer;
	for (id = 0; id < all_total_frames; id++) {
		memset(file_name, 0, sizeof(file_name));
		snprintf(file_name, sizeof(file_name), "/tmp/wvdata_%d.bin", id);
		buf = (void *)queue->wavedata_vaddr[id];
		save_as_bin_file(buf, file_name, data_len, 0);
	}
}
#endif
