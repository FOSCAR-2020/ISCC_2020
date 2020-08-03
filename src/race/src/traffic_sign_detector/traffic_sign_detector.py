#import the necessary packages
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from skimage import exposure
from skimage import feature
from imutils import paths

import argparse
import imutils
import numpy as np
import cv2

# Red Color Range
HSV_RED_LOWER = np.array([0, 100, 100])
HSV_RED_UPPER = np.array([10, 255, 255])
HSV_RED_LOWER1 = np.array([160, 100, 100])
HSV_RED_UPPER1 = np.array([179, 255, 255])

# Yellow Color Range
HSV_YELLOW_LOWER = np.array([10, 80, 120])
HSV_YELLOW_UPPER = np.array([40, 255, 255])

# Blue Color Range
HSV_BLUE_LOWER = np.array([80, 160, 65])
HSV_BLUE_UPPER = np.array([140, 255, 255])

#construct the argument parse and parse command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--training", required=True, help="Path to the logos training dataset")
args = vars(ap.parse_args())

# initialize the data matrix and labels
print ("[INFO] extracting features...")

data = []
labels = []

# loop over the image paths in the training set
for imagePath in paths.list_images(args["training"]):
	# extract the make of the car
	make = imagePath.split("/")[-2]

	# load the image, convert it to grayscale, and detect edges
	# grayscale
	image = cv2.imread(imagePath)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# resize
	sign = cv2.resize(gray, (128, 128))

	# extract Histogram of Oriented Gradients from the sign
	H = feature.hog(sign, orientations=8, pixels_per_cell=(12, 12),
		cells_per_block=(2, 2), transform_sqrt=True, block_norm="L2")

	# update the data and labels
	data.append(H)
	labels.append(make)

# "train" the nearest neighbors classifier
print("[INFO] training classifier...")
model = KNeighborsClassifier(n_neighbors=1)

model.fit(data, labels)
print(model)
print("[INFO] evaluating...")

cap = cv2.VideoCapture(0);

while True:
	ret, img = cap.read();
	cv2.imshow("frame", img)

	#gray and hsv transform
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# binary
	# 1. red
	redBinary = cv2.inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER)
	redBinary1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
	redBinary = cv2.bitwise_or(redBinary, redBinary1)

	# 2. yellow
	yellowBinary = cv2.inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)

	# 3. blue
	blueBinary = cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER)

	# red || blow
	binary = cv2.bitwise_or( blueBinary, (redBinary))

	# find contours
	image, contours, hierachy = cv2.findContours(binary, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	# 찾은 contours안을 흰색으로 채운다.(중복된 contours를 흰색으로 채워 다시 contours를 검출 한다.)
	# contours안에 contours가 잡힌 경우와 같은 영역이지만 중복해서 잡힌 경우를 예외처리한다.
	for cnt in contours:
		area = cv2.contourArea(cnt)
		binary = cv2.drawContours(binary, [cnt], -1, (255,255,255), -1)

	# 그 후 채워진 binary를 다시 contours검출
	image, goodContours, hierachy = cv2.findContours(binary, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	kernel = np.ones((3, 3), np.uint8)

	# 원본 grayscale와 binary를 and연산해여 mask를 씌워준다.
	gray = cv2.bitwise_and(binary, gray)

	# 추가 전처리 - binary mask를 팽창하여 mask의 크기를 넓혀준다.
	# gray = cv2.bitwise_and(cv2.dilate(binary, kernel, iterations = 1), gray)
	# cv2.imshow("hi", gray)

	for cnt in goodContours:
		area = cv2.contourArea(cnt)
		print(area)
		# 면적이 2000이상이고 가로/세로 비율이 0.6~1.4인 후보군을 bounding해준다.
		if area > 2000.0 :
			x, y, w, h = cv2.boundingRect(cnt)
			rate = w / h
			if rate > 0.8 and rate < 1.2 :
				cv2.rectangle(img, (x, y), (x+w, y+h), (200, 152, 50), 2)
				inputImage = gray[y:y+h, x:x+w]
				# kernel = np.ones((1, 1), np.uint8)
				# erosion = cv2.erode(inputImage, kernel, iterations=1)

				# 후보군을 resize해준 후 feature vector(Hog)를 추출해준다.
				sign = cv2.resize(inputImage, (128, 128))
				cv2.imshow("sign", sign)
				(H, hogImage) = feature.hog(sign, orientations=8, pixels_per_cell=(12, 12), \
					cells_per_block=(2, 2), transform_sqrt=True, block_norm="L2", visualise=True)

				cv2.imshow("hog", hogImage)

				# 학습된 모델에 넣는다.
				pred = model.predict(H.reshape(1, -1))[0]

				cv2.putText(img, pred.title(), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, \
					(70, 255, 0), 2)


	cv2.imshow("candidates", img)
	if cv2.waitKey(1) == 27:
		break;
