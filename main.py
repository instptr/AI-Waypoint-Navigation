from pynput.keyboard import Controller
import numpy as np
import cv2
import mss
import pygetwindow as gw
from ultralytics import YOLO
from PIL import Image
import math

PATH = "" # Path to AI model

keyboard = Controller()
model = YOLO(PATH)

window = gw.getWindowsWithTitle("Grand Theft Auto V")[0]
lastDirection = None

monitor = {
    "top": window.top + window.height - 150 - 20,
    "left": window.left + 20,
    "width": 203,
    "height": 150
}

def filterRoads(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresholded = cv2.threshold(blurred, thresh=100, maxval=255, type=cv2.THRESH_BINARY)
   
    kernel = np.ones((7,7), np.uint8)
    closed = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

    return closed

def calculateDirection(playerPos, waypointPos):
    dx = waypointPos[0] - playerPos[0]
    dy = waypointPos[1] - playerPos[1]
    angle = math.atan2(dy, dx)
    return angle

def interpolatePoints(p1, p2, numPoints=7):
    xValues = np.linspace(p1[0], p2[0], num=numPoints)
    yValues = np.linspace(p1[1], p2[1], num=numPoints)
    return list(zip(xValues, yValues))

def findClosest(point, filteredFrame):
    nonZeroPoints = np.transpose(np.nonzero(filteredFrame))
    distances = np.sqrt((nonZeroPoints[:, 0] - point[1]) ** 2 + (nonZeroPoints[:, 1] - point[0]) ** 2)

    closestPoint = np.argmin(distances)
    return nonZeroPoints[closestPoint][1], nonZeroPoints[closestPoint][0]

def log(filteredFrame, gridSize=(20, 20)):
    rows, cols = filteredFrame.shape[:2]
    gridRows, gridCols = rows // gridSize[1], cols // gridSize[0]
    loggedSquares = []

    for i in range(gridRows):
        for j in range(gridCols):
            gridStartX, gridStartY = j * gridSize[0], i * gridSize[1]
            gridEndX, gridEndY = (j + 1) * gridSize[0], (i + 1) * gridSize[1]
            gridSquare = filteredFrame[gridStartY:gridEndY, gridStartX:gridEndX]

            if np.any(gridSquare == 255):
                loggedSquares.append((j, i))

            cv2.rectangle(filteredFrame, (gridStartX, gridStartY), (gridEndX, gridEndY), (0, 255, 0), 1)

    for square in loggedSquares:
        startX, startY = square[0] * gridSize[0], square[1] * gridSize[1]
        endX, endY = (square[0] + 1) * gridSize[0], (square[1] + 1) * gridSize[1]
        cv2.rectangle(filteredFrame, (startX, startY), (endX, endY), (0, 0, 255), 2)

    return filteredFrame

def moveToRoad(playerPos, waypointPos, filteredFrame, frame):
    interpolatedPoints = interpolatePoints(playerPos, waypointPos, 7)
    roadPoints = [findClosest(point, filteredFrame) for point in interpolatedPoints]
   
    for point in roadPoints:
        cv2.circle(frame, point, radius=3, color=(0, 255, 0), thickness=-1)

    return frame

def move(direction):
    global lastDirection
    if direction != lastDirection:
        if lastDirection is not None:
            if lastDirection == "back":
                keyboard.release('s')
            elif lastDirection == "right":
                keyboard.release('d')
            elif lastDirection == "forward":
                keyboard.release('w')
            elif lastDirection == "left":
                keyboard.release('a')

        if -math.pi / 4 < direction <= math.pi / 4:
            print("right")
            keyboard.press('d')
            keyboard.press('w')
            lastDirection = "right"
        elif math.pi / 4 < direction <= 3 * math.pi / 4:
            print("back")
            keyboard.release('w')
            keyboard.press('s')
            lastDirection = "back"
        elif -3 * math.pi / 4 < direction <= -math.pi / 4:
            print("forward")
            keyboard.press('w')
            lastDirection = "forward"
        else:  # > 3*pi/4 or <= -3*pi/4
            print("left")
            keyboard.press('a')
            keyboard.press('w')
            lastDirection = "left"

def capture():
    while True:
        playerPos = None
        waypointPos = None

        with mss.mss() as sct:
            screenshot = sct.grab(monitor)
            frame = np.array(screenshot)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
           
            boxWidth, boxHeight = 5, 10
            leftOffset, bottomOffset = 15, 20

            bottomLeftCorner = (leftOffset, frame.shape[0] - boxHeight - bottomOffset)
            topRightCorner = (leftOffset + boxWidth, frame.shape[0] - bottomOffset)

            cv2.rectangle(frame, bottomLeftCorner, topRightCorner, (0, 0, 0), -1)

        filteredFrame = filterRoads(frame.copy())
        log(filteredFrame, gridSize=(20, 20))
        results = model(frame)

        for res in results:
            for box in res.boxes:
                xyxy = box.xyxy[0]
                clsId = box.cls[0].item()
               
                if clsId == 1.0:
                    playerPos = ((xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2)
                elif clsId == 0.0:
                    waypointPos = ((xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2)

        if playerPos and waypointPos:
            direction = calculateDirection(playerPos, waypointPos)
            move(direction)
            framew = moveToRoad(playerPos, waypointPos, filteredFrame, frame)

        cv2.imshow('Frame', filteredFrame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture()
