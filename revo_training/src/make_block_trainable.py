import sys, string
from keras.models import load_model

model = load_model(sys.argv[1])
block = 'block' + sys.argv[2]
for layer in model.layers:
    if layer.name.find(block) != -1:
        layer.trainable = True
    print(layer.name + ': ' + str(layer.trainable))
model.save(sys.argv[1])
