# Debugging

```
[ INFO] [1610433736.433379170, 299.063000000]: Publishing state changed to sitting
mildred_control: /usr/include/eigen3/Eigen/src/Core/Block.h:146: Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>::Block(XprType&, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index) [with XprType = Eigen::Matrix<double, -1, -1>; int BlockRows = -1; int BlockCols = -1; bool InnerPanel = false; Eigen::Index = long int]: Assertion `startRow >= 0 && blockRows >= 0 && startRow <= xpr.rows() - blockRows && startCol >= 0 && blockCols >= 0 && startCol <= xpr.cols() - blockCols' failed.
```