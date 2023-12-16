#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <windows.h>
#include <GL/gl.h>
// clang-format on

#define ZVALUE 20.0f

// OpenCV读取图像
cv::Mat I = cv::imread("D:/note/HDUFiles/computer_graphics/with_cpp/read_image/python_ko.jpg");
// 设置长宽
int width = I.cols;
int height = I.rows;
cv::Mat I2(height, width, CV_8UC3);
// 设置图像指针
GLubyte* pixels;

int window_x = width;
int window_y = height;

void get_img() {
	// OpenCV显示
	//cv::imshow("OpenCV", I);
	// 设置指针长度
	int pixellength = width * height * 3;
	// 开辟指针空间
	pixels = new GLubyte[pixellength];
	// 图像指针复制
	int components = 3;
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			int target = i + j * width, origin = i + (height - 1 - j) * width;

			for (int p = 0; p < components; p++) {
				pixels[target * components + p] = I.data[origin * components + p];
				I2.data[target * components + p] = I.data[origin * components + p];
			}
		}
	}
	// memcpy(pixels, I.data, pixellength * sizeof(char));
	//cv::imshow("OpenCV2", I2);

}

// 绘制内容
void display() {
	// 清除屏幕
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// 像素读图
	glDrawPixels(width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);
	// 双缓存交换缓存以显示图像
	glutSwapBuffers();
}

int main(int argc, char** argv) {
	// 获取图像指针函数
	get_img();
	// 初始化GL
	glutInit(&argc, argv);
	// 设置显示参数(双缓存，RGB格式)
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	// 设置窗口尺寸：width*height
	glutInitWindowSize(width, height);
	// 设置窗口位置：在屏幕左上角像素值(100,100)处
	glutInitWindowPosition(100, 100);
	// 设置窗口名称
	glutCreateWindow("OpenGL");
	// 显示函数，display事件需要自行编写
	glutDisplayFunc(display);

	// 重复循环GLUT事件
	glutMainLoop();
	// OpenGL循环结束后释放图像指针内存
	free(pixels);
}
