#   CS534 - Artificial Intelligence HMWK 4 
#   Author: <NATHANAEL> <VICKERY> 
#   Date: July 10, 2022

#   ----------------------------------------------
#   Homework 4 - Problem 5
#   Note: *This Python file shows the programming 
#         solutions to Homework 4 Problem 5. 
#   ----------------------------------------------

## Imported Libraries ----------------------------
import tensorflow as tf
from tensorflow import keras
import matplotlib.pyplot as plt
import os
import time

## Data used for both parts (a and b) ------------
plant_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data5/')
#plant_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data4/')
#plant_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data3/')
#plant_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data2/')
#plant_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data1/')
test_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data5/', batch_size=5)
validation_ds = tf.keras.utils.image_dataset_from_directory(directory='training_data5/', batch_size=10)

def process_images(image, label):
    # Normalize images to have a mean of 0 and standard deviation of 1
    image = tf.image.per_image_standardization(image)
    # Resize images from 32x32 to 277x277
    image = tf.image.resize(image, (227,227))
    return image, label

plant_ds_size = tf.data.experimental.cardinality(plant_ds).numpy()
test_ds_size = tf.data.experimental.cardinality(test_ds).numpy()
validation_ds_size = tf.data.experimental.cardinality(validation_ds).numpy()
print("Training data size:", plant_ds_size)
print("Test data size:", test_ds_size)
print("Validation data size:", validation_ds_size)

plant_ds = (plant_ds
                  .map(process_images)
                  .shuffle(buffer_size=plant_ds_size))
                #  .batch(batch_size=32, drop_remainder=True))
test_ds = (test_ds
                  .map(process_images)
                  .shuffle(buffer_size=plant_ds_size)
                  .batch(batch_size=32, drop_remainder=True))
validation_ds = (validation_ds
                  .map(process_images)
                  .shuffle(buffer_size=plant_ds_size))
              #    .batch(batch_size=32, drop_remainder=True))

## Alexnet Model ---------------------------------
model = keras.models.Sequential([
    keras.layers.Conv2D(filters=96, kernel_size=(11,11), strides=(4,4), activation='relu', input_shape=(227,227,3)),
    keras.layers.BatchNormalization(),
    keras.layers.MaxPool2D(pool_size=(3,3), strides=(2,2)),
    keras.layers.Conv2D(filters=256, kernel_size=(5,5), strides=(1,1), activation='relu', padding="same"),
    keras.layers.BatchNormalization(),
    keras.layers.MaxPool2D(pool_size=(3,3), strides=(2,2)),
    keras.layers.Conv2D(filters=384, kernel_size=(3,3), strides=(1,1), activation='relu', padding="same"),
    keras.layers.BatchNormalization(),
    keras.layers.Conv2D(filters=384, kernel_size=(3,3), strides=(1,1), activation='relu', padding="same"),
    keras.layers.BatchNormalization(),
    keras.layers.Conv2D(filters=256, kernel_size=(3,3), strides=(1,1), activation='relu', padding="same"),
    keras.layers.BatchNormalization(),
    keras.layers.MaxPool2D(pool_size=(3,3), strides=(2,2)),
    keras.layers.Flatten(),
    keras.layers.Dense(4096, activation='relu'),
    keras.layers.Dropout(0.5),
    keras.layers.Dense(4096, activation='relu'),
    keras.layers.Dropout(0.5),
    keras.layers.Dense(10, activation='softmax')
])

model.compile(loss='sparse_categorical_crossentropy', optimizer=tf.optimizers.SGD(lr=0.001), metrics=['accuracy'])
model.summary()

## Model Training -------------------------------
root_logdir = os.path.join(os.curdir, "logs\\fit\\")
def get_run_logdir():
    run_id = time.strftime("run_%Y_%m_%d-%H_%M_%S")
    return os.path.join(root_logdir, run_id)
run_logdir = get_run_logdir()
tensorboard_cb = keras.callbacks.TensorBoard(run_logdir)

epochs_count = 25

history = model.fit(plant_ds,
          epochs=epochs_count,
          validation_data=validation_ds,
          validation_freq=1,
          callbacks=[tensorboard_cb])


print(history.history.keys())

acc = history.history['accuracy']
val_acc = history.history['val_accuracy']



epochs_range = range(epochs_count)

plt.figure(figsize=(8, 8))
plt.plot(epochs_range, acc, label='Training Accuracy')
plt.plot(epochs_range, val_acc, label='Validation Accuracy')
plt.legend(loc='lower right')
plt.title('Training and Validation Accuracy for 100 Images')
#plt.title('Training and Validation Accuracy for 150 Images')
#plt.title('Training and Validation Accuracy for 200 Images')
#plt.title('Training and Validation Accuracy for 250 Images')
#plt.title('Training and Validation Accuracy for 300 Images')
plt.xlabel('Epochs')
plt.ylabel('Accuracy')
plt.show()
