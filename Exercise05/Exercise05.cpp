//プログラムの解説
//操作方法：
////w,s,a,d,q,eでx軸正負,y軸正負,Z軸正負の方向に平行移動、
//z,x,c,v,b,nでX軸正負,Y軸正負,Z軸正負周りに回転

#include <iostream>
#include <GL/glut.h>
#include <math.h>

//変数定義

//変換についての変数
float px = 0.3; //図形の描写位置 x座標
float py = 0.3; //図形の描写位置 y座標
float pz = 0.3; //図形の描写位置 z座標
float ax = 0.1; //X軸周りの図形の回転角
float ay = 0.3; //Y軸周りの図形の回転角
float az = 0.6; //Z軸周りの図形の回転角

//視点についての変数
double s = 1.0; //視点から投影面までの距離
double d = 1.9; //視点から原点までの距離

//変形させる対象（2次元座標系内の正方形）
int nv = 8; //頂点数
double model_v[][3] = { //立方体の3次元頂点の座標
	{0.0, 0.5, 0.0}, //0
	{0.0, 0.0, 0.0}, //1
	{0.5, 0.0, 0.0}, //2
	{0.5, 0.5, 0.0}, //3
	{0.0, 0.5, 0.5}, //4
	{0.0, 0.0, 0.5}, //5
	{0.5, 0.0, 0.5}, //6
	{0.5, 0.5, 0.5}  //7
};

int model_f[][4] = { //面が使う頂点（外から見て逆時計回り）
	{1, 0, 3, 2}, //第0面
	{5, 1, 2, 6}, //第1面
	{4, 5, 6, 7}, //第2面
	{7, 3, 0, 4}, //第3面
	{0, 1, 5, 4}, //第4面
	{2, 3, 7, 6}, //第5面
};

int model_c[][3] = { //立方体の各面の色
	{1.0, 1.0, 0.0}, //黄
	{1.0, 0.0, 0.0}, //赤
	{0.0, 1.0, 1.0}, //水色
	{0.0, 1.0, 0.0}, //緑
	{1.0, 0.0, 1.0}, //ピンク
	{0.0, 0.0, 1.0}  //青
};

#define PMAX 100 //扱うことができる頂点数の最大数
double vertices[PMAX][3]; //動的に確保すべきだが簡単のためにこのようにする

//変換行列
double transformation_mat[4][4]; //変換の合成

//X軸まわりの回転を行う回転行列を生成する関数
void makeRotation3DXMatrix(double mat[4][4], const double r) {
	mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0; mat[0][3] = 0;
	mat[1][0] = 0; mat[1][1] = cos(r); mat[1][2] = -sin(r); mat[1][3] = 0;
	mat[2][0] = 0; mat[2][1] = sin(r); mat[2][2] = cos(r); mat[2][3] = 0;
	mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
};

//Y軸まわりの回転を行う回転行列を生成する関数
void makeRotation3DYMatrix(double mat[4][4], const double r) {
	mat[0][0] = cos(r); mat[0][1] = 0; mat[0][2] = sin(r); mat[0][3] = 0;
	mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0; mat[1][3] = 0;
	mat[2][0] = -sin(r); mat[2][1] = 0; mat[2][2] = cos(r); mat[2][3] = 0;
	mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
};

//Z軸まわりの回転を行う回転行列を生成する関数
void makeRotation3DZMatrix(double mat[4][4], const double r) {
	mat[0][0] = cos(r); mat[0][1] = -sin(r); mat[0][2] = 0; mat[0][3] = 0;
	mat[1][0] = sin(r); mat[1][1] = cos(r); mat[1][2] = 0; mat[1][3] = 0;
	mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1; mat[2][3] = 0;
	mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
};

//任意の平行移動を実現する同次変換行列を作成する関数
// tx, ty, tzはx, y, z座標の移動量とし，mat[][]は返り値であり生成された変換行列が代入されるもの
void makeTranslation3DMatrix(double mat[4][4], const double tx, const double ty, const double tz) {
	mat[0][0] = 1; mat[0][1] = 0; mat[0][2] = 0; mat[0][3] = tx;
	mat[1][0] = 0; mat[1][1] = 1; mat[1][2] = 0; mat[1][3] = ty;
	mat[2][0] = 0; mat[2][1] = 0; mat[2][2] = 1; mat[2][3] = tz;
	mat[3][0] = 0; mat[3][1] = 0; mat[3][2] = 0; mat[3][3] = 1;
};

//同次変換行列の積を求めて，合成変換を行う同次変換行列を求める関数
void makeTransformation3DMatrix(double out_mat[4][4], const double in_mat1[4][4], const double in_mat2[4][4]) {
	double tmp_mat[4][4];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			tmp_mat[i][j] =
				in_mat1[i][0] * in_mat2[0][j] + in_mat1[i][1] * in_mat2[1][j] + in_mat1[i][2] * in_mat2[2][j] + in_mat1[i][3] * in_mat2[3][j];
		}
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			out_mat[i][j] = tmp_mat[i][j];
		}
	}
}

//３次元のX軸まわりの回転変換を変換行列に適用する関数
void append3DRotationX(const double r) {
	double rotation_matX[4][4]; //回転行列を格納する配列を定義
	makeRotation3DXMatrix(rotation_matX, r); //rの方向に回転移動させる、rotation_matXは返り値であり生成された変換行列が代入される
	makeTransformation3DMatrix(transformation_mat, rotation_matX, transformation_mat); //rotation_matとtransformation_matの行列の積の計算を行い、その結果をtransformation_matに代入する
};
//３次元のY軸まわりの回転変換を変換行列に適用する関数
void append3DRotationY(const double r) {
	double rotation_matY[4][4]; //回転行列を格納する配列を定義
	makeRotation3DYMatrix(rotation_matY, r); //rの方向に回転移動させる、rotation_matYは返り値であり生成された変換行列が代入される
	makeTransformation3DMatrix(transformation_mat, rotation_matY, transformation_mat); //rotation_matとtransformation_matの行列の積の計算を行い、その結果をtransformation_matに代入する
};
//３次元のZ軸まわりの回転変換を変換行列に適用する関数
void append3DRotationZ(const double r) {
	double rotation_matZ[4][4]; //回転行列を格納する配列を定義
	makeRotation3DZMatrix(rotation_matZ, r); //rの方向に回転移動させる、rotation_matZは返り値であり生成された変換行列が代入される
	makeTransformation3DMatrix(transformation_mat, rotation_matZ, transformation_mat); //rotation_matとtransformation_matの行列の積の計算を行い、その結果をtransformation_matに代入する
}

//現在の合成変換に平行移動を追加する関数
//x, y, z軸正の方向にtx, ty, tz平行移動する、mat[][]は返り値であり生成された（平行移動の情報を持つ）変換行列が代入される
void appendTranslation3D(double mat[4][4], const double tx, const double ty, const double tz) {
	double operation_mat[4][4]; //平行移動行列を格納する配列を定義
	makeTranslation3DMatrix(operation_mat, tx, ty, tz); // tx, tyはx座標とy座標の移動量とし，operation_matは返り値であり生成された変換行列が代入される
	makeTransformation3DMatrix(mat, operation_mat, mat); //operation_matとtransformation_matの行列の積の計算を行い、その結果をtransformation_matに代入する
}

//mat[4][4]を単位行列にする関数
void setIdentity3D(double mat[4][4]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				mat[i][j] = 1.0;
			}
			else {
				mat[i][j] = 0.0;
			}
		}
	}
}

//4*4行列Aと4*1行列vのかけ算Avの計算
//同次変換行列を用いてモデルの頂点座標model_v[]を変換する関数
//返り値：vertices[] = transformation_mat * model_v[]
void transform3D(double vertices[], double model_v[], const double mat[4][4]) {
	double temp_x = model_v[0];
	double temp_y = model_v[1];
	double temp_z = model_v[2];
	for (int i = 0; i < 3; i++) {
		vertices[i] = temp_x * mat[i][0] + temp_y * mat[i][1] + temp_z * mat[i][2] + 1.0 * mat[i][3];
	}
}

//描写用コールバック関数
void display(void) {
	glClear(GL_COLOR_BUFFER_BIT); //描画用のバッファを初期化（クリア＝塗りつぶし）する

	//座標軸の表示
	glColor3f(1.0, 1.0, 1.0); //白
	glBegin(GL_LINES); //直線（プリミティブ）を打つ この関数は、glClearとglFlushの間に書く
	glVertex2d(-1.0, 0.0);
	glVertex2d(1.0, 0.0);
	glVertex2d(0.0, -1.0);
	glVertex2d(0.0, 1.0);
	glEnd(); //glBeginに対応する関数

	//変換行列の生成
	setIdentity3D(transformation_mat); //transformation_matを単位行列にする
	append3DRotationX(ax); //X軸回りの回転変換を追加する
	append3DRotationY(ay); //Y軸回りの回転変換を追加する
	append3DRotationZ(az); //Z軸回りの回転変換を追加する
	appendTranslation3D(transformation_mat, px, py, pz); //並進変換を追加する

	//同次変換行列を用いてモデルの各頂点座標model_v[]を変換する
	//座標変換
	for (int i = 0; i < nv; i++) {
		transform3D(vertices[i], model_v[i], transformation_mat);
	};

	//立方体の表示
	glColor3f(0.0, 1.0, 0.0); //緑
	glBegin(GL_POINTS); //点のループ（プリミティブ）を打つ この関数は、glClearとglFlushの間に書く
	for (int i = 0; i < nv; i++) {
		glVertex2d(s / (d - vertices[i][2]) * vertices[i][0], s / (d - vertices[i][2]) * vertices[i][1]);
	}
	glEnd(); //glBeginに対応する関数

	//立方体の面の表示
	for (int i = 0; i < 6; i++) { //立方体の1面を描写
		glColor3f(model_c[i][0], model_c[i][1], model_c[i][2]); //立方体の各面の色を指定
		glBegin(GL_QUADS); //四角形（プリミティブ）を打つ この関数は、glClearとglFlushの間に書く
		for (int j = 0; j < 4; j++) { //各面の4頂点
			glVertex2d(s / (d - vertices[model_f[i][j]][2]) * vertices[model_f[i][j]][0], s / (d - vertices[model_f[i][j]][2]) * vertices[model_f[i][j]][1]);
		}
		glEnd(); //glBeginに対応する関数
	}


	glFlush(); //コマンドが全て実行され、描画（レンダリング）する
}

//キーボードを入力したときに図形の位置や回転角を変更する関数
void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'w': //上
		py += 0.1;
		glutPostRedisplay();
		break;
	case 'a': //左
		px -= 0.1;
		glutPostRedisplay();
		break;
	case 'd': //右
		px += 0.1;
		glutPostRedisplay();
		break;
	case 's': //下
		py -= 0.1;
		glutPostRedisplay();
		break;
	case 'q': //天
		pz += 0.1;
		glutPostRedisplay();
		break;
	case 'e': //地
		pz -= 0.1;
		glutPostRedisplay();
		break;
	case 'z': //X軸周り正の方向の回転角
		ax += 0.1;
		glutPostRedisplay();
		break;
	case 'x': //X軸周り負の方向の回転角
		ax -= 0.1;
		glutPostRedisplay();
		break;
	case 'c': //Y軸周り正の方向の回転角
		ay += 0.1;
		glutPostRedisplay();
		break;
	case 'v': //Y軸周り負の方向の回転角
		ay -= 0.1;
		glutPostRedisplay();
		break;
	case 'b': //Z軸周り正の方向の回転角
		az += 0.1;
		glutPostRedisplay();
		break;
	case 'n': //Z軸周り負の方向の回転角
		az -= 0.1;
		glutPostRedisplay();
		break;
	default: //それ以外
		break;
	}
}

int main(int argc, char *argv[]) {
	glutInit(&argc, argv); //glutの初期化
	glutInitDisplayMode(GLUT_RGBA); //RGBAモードで描画
	glutCreateWindow(argv[0]); //ウィンドウの作成
	glClearColor(0.0, 0.0, 0.0, 0.0); //ウィンドウを塗りつぶす色を指定
	glutDisplayFunc(display); //コールバック関数を呼び出し処理を行う
	glutKeyboardFunc(keyboard); //キーボード入力のコールバック関数
	glutMainLoop(); //繰り返す

	return 0;
}