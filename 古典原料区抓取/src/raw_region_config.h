#ifndef RAW_REGION_CONFIG_H
#define RAW_REGION_CONFIG_H

/* 相机 HDISC2 和 J 坐标的有效图像尺寸。尺寸变更时须同步相机输出，并复测
 * station_config.h 的三个原料分区和视觉换算比例。 */
#define RAW_J_IMAGE_WIDTH  640 /* 图像宽度，px；有效 X 为 0 到宽度减 1。 */
#define RAW_J_IMAGE_HEIGHT 480 /* 图像高度，px；有效 Y 为 0 到高度减 1。 */

#endif
