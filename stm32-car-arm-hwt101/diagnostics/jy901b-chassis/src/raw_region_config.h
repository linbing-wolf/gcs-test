#ifndef RAW_REGION_CONFIG_H
#define RAW_REGION_CONFIG_H
/* 直接使用相机回包坐标，不按照片显示方向翻转Y。范围端点均包含。
 * 每项依次为 X最小、X最大、Y最小、Y最大，单位像素。
 * 用户最新样点：左(136,44)、中(310,315)、右(491,32)。
 * 粗分试调初值：X按两侧样点中点取314，Y按较高侧点44与中点315中点取180。
 * 这些范围由样点推导，并非完整可抓范围实测；固定相机姿态后再验证。
 * -1仍可表示未标定。 */
#define RAW_J_RIGHT_REGION  { 314, 639,   0, 179 }
#define RAW_J_MIDDLE_REGION {   0, 639, 180, 479 }
#define RAW_J_LEFT_REGION   {   0, 313,   0, 179 }
#define RAW_J_IMAGE_WIDTH   640
#define RAW_J_IMAGE_HEIGHT  480
#endif
