//
// Created by kinit on 2021-04-23.
//

#include "TestClassUtil.h"
#include "../ipc/ClassificationManager.h"
#include "../util/common.h"

#include "../ui/VgaFont.h"
#include "../ui/Widgets.h"
#include <dirent.h>

#define TEST_WIN_NAME "detection"

using namespace std;
using namespace cv;

void runHeadlessTest(const char *basePath, int type) {
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(basePath)) == nullptr) {
        printf("目录名不正确/n");
        return;
    }
    int num = 0;
    int success = 0;
    int fail = 0;
    int miss = 0;
    char filepath[256];
    while ((dirp = readdir(dp)) != nullptr) {
        if (strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0) {
            continue;
        }
        sprintf(filepath, "%s/%s", basePath, dirp->d_name);
        {
            char buf[64];
            Mat img = imread(filepath);
            auto result = ClassificationManager::classify(img);
            const char *label = ClassificationManager::getLabelName(result.type);
            if (result.type == ClassificationManager::TYPE_NOTHING) {
                miss++;
            } else if (result.type == type) {
                success++;
            } else {
                fail++;
            }
            DrawTextLeftCenterAutoColor(img, (sprintf(buf, "%s %0.3f", label, result.confidence), buf), 16, 16);
            imshow(TEST_WIN_NAME, img);
            waitKey(1);
        }
        num++;
    }
    printf("\n\n");
    printf("%.3f(%d) / %.3f(%d) / %.3f(%d)",
           100.0f * success / num, success, 100.0f * fail / num, fail, 100.0f * miss / num, miss);
    printf("\n\n");
}

void showTestWindowLoop() {
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/square/1618987771111.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/square/1618987771952.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/square/1618987771704.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/square/1618987784156.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/round/1618988581262.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/round/1618988584979.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/round/1618988635578.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/round/1618988723685.jpg";
//    const char *imgPath = "/home/kinit/CLionProjects/Abs_96x96/nothing/1618988970853.jpg";



    const char *squarePath = "/home/kinit/CLionProjects/Abs_96x96/square";
    const char *roundPath = "/home/kinit/CLionProjects/Abs_96x96/round";
    const char *nothingPath = "/home/kinit/CLionProjects/Abs_96x96/nothing";

    runHeadlessTest(squarePath, ClassificationManager::TYPE_SQUARE);
    runHeadlessTest(roundPath, ClassificationManager::TYPE_ROUND);
    runHeadlessTest(nothingPath, ClassificationManager::TYPE_NOTHING);
}


