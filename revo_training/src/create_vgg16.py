from keras.models import Model
from keras.applications.vgg16 import VGG16
from keras.layers import Flatten, Dropout, Dense
from keras.utils import plot_model

vgg = VGG16(weights='imagenet', include_top=False, input_shape=(224,224,3))
print(vgg.layers)
for layer in vgg.layers:
    layer.trainable = False

flat = Flatten()(vgg.output)
fc6 = Dense(1024, activation='relu', name='fc6')(flat)
drop1 = Dropout(0.5)(fc6)
fc7 = Dense(1024, activation='relu', name='fc7')(drop1)
drop2 = Dropout(0.5)(fc7)
classify = Dense(3, activation='softmax', name='classify')(drop2)

model = Model(vgg.input, classify)
model.save('VGG16-new.h5')
plot_model(model, 'VGG16-new_plot.png', show_shapes=True)
