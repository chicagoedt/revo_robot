# 0.05 sec/img, converges to ~0.98 val_acc
def buildModelA():
    model = Sequential()
    model.add(Conv2D(64, (11,11), padding='same', activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D())
    model.add(Conv2D(128, (5,5), padding='same', activation='relu'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(128, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Dropout(0.5))
    model.add(Conv2D(512, (7,7), padding='same', activation='relu', dilation_rate=4))
    model.add(Dropout(0.25))
    model.add(Conv2D(1024, (1,1), padding='same', activation='relu'))
    model.add(Conv2D(1, (1,1), padding='same', activation='sigmoid'))
    return model

# 0.006 sec/img, converges to ~0.9754 val_acc
def buildModelB1():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(Dropout(0.25)(conv2_1))
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(Dropout(0.25)(conv2_2))
    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv2_3)

    return Model(inputs=img, outputs=pred)

# 0.0065 sec/img
def buildModelB2():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv3_3)

    return Model(inputs=img, outputs=pred)

# 0.0128 sec/img, converges to 0.945 val_acc
def buildModelB3():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same')(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same')(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same')(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same')(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same')(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same')(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.013 sec/img, converges to ~0.977 val_acc
def buildModelB4():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same')(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same')(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.016 sec/img, converges to ~0.983 val_acc
def buildModelB5():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=8, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=16, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.0163 sec/img, converges to ~0.985 val_acc
def buildModelB6():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=8, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(Dropout(0.25)(conv5_3))
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=16, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(Dropout(0.5)(conv6_3))

    return Model(inputs=img, outputs=pred)

def buildModelB6_1():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(128, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(128, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(128, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5_1 = Conv2D(128, (3,3), padding='same', dilation_rate=8)(conv4_3)
    conv5_2 = Conv2D(128, (3,3), padding='same', dilation_rate=8)(conv5_1)
    conv5_3 = Conv2D(128, (3,3), padding='same', dilation_rate=8, activation='relu')(conv5_2)

    conv6_1 = Conv2D(128, (3,3), padding='same', dilation_rate=16)(Dropout(0.25)(conv5_3))
    conv6_2 = Conv2D(128, (3,3), padding='same', dilation_rate=16)(conv6_1)
    conv6_3 = Conv2D(128, (3,3), padding='same', dilation_rate=16, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(Dropout(0.5)(conv6_3))

    return Model(inputs=img, outputs=pred)
# 0.011 sec/img
def buildModelC1():
    img = Input(shape=input_shape)

    conv1 = Conv2D(8, (1,1), padding='same')(img)
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (5,5), padding='same')(pool1)
    conv2_2 = Conv2D(64, (5,5), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (5,5), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (5,5), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (5,5), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (5,5), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (5,5), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (5,5), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (5,5), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5 = Conv2D(128, (11,11), padding='same', dilation_rate=2, activation='relu')(conv4_3)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(Dropout(0.5)(conv5))

    return Model(inputs=img, outputs=pred)


# 
def buildModelD1():
    img = Input(shape=input_shape)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(img)
    pool1 = MaxPooling2D()(conv1)

    conv2 = Conv2D(32, (7,7), padding='same', activation='relu')(pool1)
    pool2 = MaxPooling2D()(conv2)

    conv3 = Conv2D(32, (11,11), padding='same', dilation_rate=2, activation='relu')(pool2)

    pred = Conv2D(1, (1,1), padding='same', activation='sigmoid')(conv3)

    return Model(inputs=img, outputs=pred)