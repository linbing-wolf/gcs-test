# 分类数据集划分

```
train/<类别>/   80%
val/<类别>/     20%
calib/         量化校准集(每类10张)
```

- 训练：MaixHub 分类任务 / ultralytics `yolo classify`，直接指向本目录
- `calib/`：把图片打包成 zip 作为 MaixCAM2 INT8 量化的校准集
- 类别顺序：background, blue, green, other, red, yellow
