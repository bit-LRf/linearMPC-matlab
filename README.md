# linearMPC-matlab
本函数中的变量未说明下默认按列向量处理。

在本案例中，系统为dx/dt = u^3

主要函数：
1.核心功能函数：mpcOptmizer：输入状态量，控制量，可行解，观测参考量。输出最优解，最优解处的值，成功求解标记，最优解处的观测量。
2.getParameter：给参数的。
3.getReference：给观测参考量的。
4.getNextstate：输入状态量，控制量。输出下一个状态量。
5.getDiscreteMatrix：输入状态量，控制量。输出线性化且离散化后的状态空间方程中的矩阵A、B、C：X(k + 1) = AX(k) + Bu(k) + C
6.mpcFunction：在本案例中使用的是simulink仿真和s函数，此函数就是s函数。
7.differentialFunction：原始的状态空间方程，输入状态量和控制量，输出状态量导数
8.trajectorySynthesis：合成轨迹，输入控制量序列组合，输出一个控制量序列

附加函数：
1.motionMatrixLinearlization；线性化的工具，并不随主函数运行
2.debug：debug用的，可以调用主函数

修改模型时需要修改的函数：
1.getParameter：权重和观测矩阵，控制量和状态量相关参数
2.getReference：参考值获取
3.differentialFunction：原始状态空间方程
4.getDiscreteMatrix：雅克比线性化模型部分