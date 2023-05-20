# linearMPC-matlab
主要函数：
1.核心功能函数：mpcOptmizer：输入状态量，控制量，可行解，观测参考量。输出最优解，最优解处的值，成功求解标记，最优解处的观测量。
2.getParameter：给参数的。
3.getReference：给观测参考量的。
4.getNextstate：调用格式请与原来一致：输入状态量，控制量。输出下一个状态量。
5.getDiscreteMatrix：输入状态量，控制量。输出线性化且离散化后的状态空间方程中的矩阵ABC（X(k + 1) = AX(k) + Bu(k) + C）
6.mpcFunction：在本案例中使用的是simulink仿真和s-函数，此函数就是s-函数。
