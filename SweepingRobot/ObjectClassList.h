#ifndef __OBJECTCLASSLIST_H__
#define __OBJECTCLASSLIST_H__

struct ObjectDetectionItem {
    uint8_t index;
    const char* objectName;
    uint8_t filter;
};

// List of objects the pre-trained model is capable of recognizing
// Index number is fixed and hard-coded from training
// Set the filter value to 0 to ignore any recognized objects
ObjectDetectionItem itemList[80] = {
    {0,  "person",         1},
    {1,  "bicycle",        0},
    {2,  "car",            0},
    {3,  "motorbike",      0},
    {4,  "aeroplane",      0},
    {5,  "bus",            0},
    {6,  "train",          0},
    {7,  "truck",          0},
    {8,  "boat",           0},
    {9,  "traffic light",  0},
    {10, "fire hydrant",   0},
    {11, "stop sign",      0},
    {12, "parking meter",  0},
    {13, "bench",          0},
    {14, "bird",           0},
    {15, "cat",            0},
    {16, "dog",            0},
    {17, "horse",          0},
    {18, "sheep",          0},
    {19, "cow",            0},
    {20, "elephant",       0},
    {21, "bear",           0},
    {22, "zebra",          0},
    {23, "giraffe",        0},
    {24, "backpack",       0},
    {25, "umbrella",       0},
    {26, "handbag",        0},
    {27, "tie",            0},
    {28, "suitcase",       0},
    {29, "frisbee",        0},
    {30, "skis",           0},
    {31, "snowboard",      0},
    {32, "sports ball",    0},
    {33, "kite",           0},
    {34, "baseball bat",   0},
    {35, "baseball glove", 0},
    {36, "skateboard",     0},
    {37, "surfboard",      0},
    {38, "tennis racket",  0},
    {39, "bottle",         0},
    {40, "wine glass",     0},
    {41, "cup",            0},
    {42, "fork",           0},
    {43, "knife",          0},
    {44, "spoon",          0},
    {45, "bowl",           0},
    {46, "banana",         0},
    {47, "apple",          0},
    {48, "sandwich",       0},
    {49, "orange",         0},
    {50, "broccoli",       0},
    {51, "carrot",         0},
    {52, "hot dog",        0},
    {53, "pizza",          0},
    {54, "donut",          0},
    {55, "cake",           0},
    {56, "chair",          0},
    {57, "sofa",           0},
    {58, "pottedplant",    0},
    {59, "bed",            0},
    {60, "diningtable",    0},
    {61, "toilet",         0},
    {62, "tvmonitor",      0},
    {63, "laptop",         0},
    {64, "mouse",          0},
    {65, "remote",         0},
    {66, "keyboard",       0},
    {67, "cell phone",     0},
    {68, "microwave",      0},
    {69, "oven",           0},
    {70, "toaster",        0},
    {71, "sink",           0},
    {72, "refrigerator",   0},
    {73, "book",           0},
    {74, "clock",          0},
    {75, "vase",           0},
    {76, "scissors",       0},
    {77, "teddy bear",     0},
    {78, "hair dryer",     0},
    {79, "toothbrush",     0},
};

#endif
