[basic]
type = axmodel
model_npu = cls11n_npu.axmodel
model_vnpu = cls11n_vnpu.axmodel

[extra]
model_type = classifier
input_type = rgb
labels = background, blue, green, other, red, yellow
input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true
mean = 0,0,0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098