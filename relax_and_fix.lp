\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0.9 Ix + 0.4 Ix + 0.8 Ix + 0.5 Ix + 0.1 Ix + 0.9 Ix + 0.4 Ix + 0.8 Ix
   + 0.5 Ix + 0.1 Ix + 0.9 Ix + 0.4 Ix + 0.8 Ix + 0.5 Ix + 0.1 Ix + 0.9 Ix
   + 0.4 Ix + 0.8 Ix + 0.5 Ix + 0.1 Ix + 0.9 Ix + 0.4 Ix + 0.8 Ix + 0.5 Ix
   + 0.1 Ix + 0.9 Ix + 0.4 Ix + 0.8 Ix + 0.5 Ix + 0.1 Ix + 0.9 Ix + 0.4 Ix
   + 0.8 Ix + 0.5 Ix + 0.1 Ix + 0.9 Ix + 0.4 Ix + 0.8 Ix + 0.5 Ix + 0.1 Ix
   + 0.9 stock_x[1,9] + 0.4 stock_x[2,9] + 0.8 stock_x[3,9]
   + 0.5 stock_x[4,9] + 0.1 stock_x[5,9] + 4.5 y[1,9] + 6.2 y[2,9]
   + 4.2 y[3,9] + 4 y[4,9] + 9.5 y[5,9] + 0.9 stock_x[1,10]
   + 0.4 stock_x[2,10] + 0.8 stock_x[3,10] + 0.5 stock_x[4,10]
   + 0.1 stock_x[5,10] + 4.5 y[1,10] + 6.2 y[2,10] + 4.2 y[3,10]
   + 4 y[4,10] + 9.5 y[5,10] + 113.4 Constant
Subject To
 u_constrs[1,1]: Ix - x = -1
 u_constrs[1,2]: - Ix + Ix - x = -6
 u_constrs[1,3]: - Ix + Ix - x = -10
 u_constrs[1,4]: - Ix + Ix - x = -6
 u_constrs[1,5]: - Ix + Ix - x = -5
 u_constrs[1,6]: - Ix + Ix - x = -7
 u_constrs[1,7]: - Ix + Ix - x = -5
 u_constrs[1,8]: - Ix + Ix - x = -9
 u_constrs[1,9]: - Ix - x[1,9] + stock_x[1,9] = -4
 u_constrs[1,10]: - stock_x[1,9] - x[1,10] + stock_x[1,10] = -5
 u_constrs[2,1]: Ix - x = -2
 u_constrs[2,2]: - Ix + Ix - x = -9
 u_constrs[2,3]: - Ix + Ix - x = 0
 u_constrs[2,4]: - Ix + Ix - x = 0
 u_constrs[2,5]: - Ix + Ix - x = 0
 u_constrs[2,6]: - Ix + Ix - x = 0
 u_constrs[2,7]: - Ix + Ix - x = -8
 u_constrs[2,8]: - Ix + Ix - x = -8
 u_constrs[2,9]: - Ix - x[2,9] + stock_x[2,9] = -2
 u_constrs[2,10]: - stock_x[2,9] - x[2,10] + stock_x[2,10] = -9
 u_constrs[3,1]: Ix - x = -3
 u_constrs[3,2]: - Ix + Ix - x = -4
 u_constrs[3,3]: - Ix + Ix - x = -7
 u_constrs[3,4]: - Ix + Ix - x = 0
 u_constrs[3,5]: - Ix + Ix - x = -2
 u_constrs[3,6]: - Ix + Ix - x = -7
 u_constrs[3,7]: - Ix + Ix - x = -2
 u_constrs[3,8]: - Ix + Ix - x = -4
 u_constrs[3,9]: - Ix - x[3,9] + stock_x[3,9] = -4
 u_constrs[3,10]: - stock_x[3,9] - x[3,10] + stock_x[3,10] = -10
 u_constrs[4,1]: Ix - x = -4
 u_constrs[4,2]: - Ix + Ix - x = -4
 u_constrs[4,3]: - Ix + Ix - x = -9
 u_constrs[4,4]: - Ix + Ix - x = -4
 u_constrs[4,5]: - Ix + Ix - x = -8
 u_constrs[4,6]: - Ix + Ix - x = -9
 u_constrs[4,7]: - Ix + Ix - x = -3
 u_constrs[4,8]: - Ix + Ix - x = -4
 u_constrs[4,9]: - Ix - x[4,9] + stock_x[4,9] = -8
 u_constrs[4,10]: - stock_x[4,9] - x[4,10] + stock_x[4,10] = -6
 u_constrs[5,1]: Ix - x = -5
 u_constrs[5,2]: - Ix + Ix - x = -8
 u_constrs[5,3]: - Ix + Ix - x = -3
 u_constrs[5,4]: - Ix + Ix - x = -3
 u_constrs[5,5]: - Ix + Ix - x = -10
 u_constrs[5,6]: - Ix + Ix - x = -8
 u_constrs[5,7]: - Ix + Ix - x = -8
 u_constrs[5,8]: - Ix + Ix - x = -7
 u_constrs[5,9]: - Ix - x[5,9] + stock_x[5,9] = -9
 u_constrs[5,10]: - stock_x[5,9] - x[5,10] + stock_x[5,10] = -2
 v_constrs[1]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 13.9
 v_constrs[2]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 27.8
 v_constrs[3]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 24.9
 v_constrs[4]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 38.8
 v_constrs[5]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 36.5
 v_constrs[6]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 24.9
 v_constrs[7]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 32.1
 v_constrs[8]: 0.8 x + 0.6 x + 0.7 x + 0.2 x + 0.5 x <= 24.9
 v_constrs[9]: 0.8 x[1,9] + 0.6 x[2,9] + 0.7 x[3,9] + 0.2 x[4,9]
   + 0.5 x[5,9] + 9.2 y[1,9] + 6.7 y[2,9] + 6.7 y[3,9] + 7.2 y[4,9]
   + 4.3 y[5,9] <= 48
 v_constrs[10]: 0.8 x[1,10] + 0.6 x[2,10] + 0.7 x[3,10] + 0.2 x[4,10]
   + 0.5 x[5,10] + 9.2 y[1,10] + 6.7 y[2,10] + 6.7 y[3,10] + 7.2 y[4,10]
   + 4.3 y[5,10] <= 48
 w_constrs[1,1]: x <= 48.5
 w_constrs[1,2]: x <= 48.5
 w_constrs[1,3]: x <= 48.5
 w_constrs[1,4]: x <= 48.5
 w_constrs[1,5]: x <= 0
 w_constrs[1,6]: x <= 48.5
 w_constrs[1,7]: x <= 48.5
 w_constrs[1,8]: x <= 48.5
 w_constrs[1,9]: x[1,9] - 48.5 y[1,9] <= 0
 w_constrs[1,10]: x[1,10] - 48.5 y[1,10] <= 0
 w_constrs[2,1]: x <= 38
 w_constrs[2,2]: x <= 0
 w_constrs[2,3]: x <= 0
 w_constrs[2,4]: x <= 0
 w_constrs[2,5]: x <= 0
 w_constrs[2,6]: x <= 0
 w_constrs[2,7]: x <= 38
 w_constrs[2,8]: x <= 0
 w_constrs[2,9]: x[2,9] - 38 y[2,9] <= 0
 w_constrs[2,10]: x[2,10] - 38 y[2,10] <= 0
 w_constrs[3,1]: x <= 43
 w_constrs[3,2]: x <= 43
 w_constrs[3,3]: x <= 43
 w_constrs[3,4]: x <= 0
 w_constrs[3,5]: x <= 0
 w_constrs[3,6]: x <= 43
 w_constrs[3,7]: x <= 0
 w_constrs[3,8]: x <= 43
 w_constrs[3,9]: x[3,9] - 43 y[3,9] <= 0
 w_constrs[3,10]: x[3,10] - 43 y[3,10] <= 0
 w_constrs[4,1]: x <= 59
 w_constrs[4,2]: x <= 0
 w_constrs[4,3]: x <= 59
 w_constrs[4,4]: x <= 0
 w_constrs[4,5]: x <= 59
 w_constrs[4,6]: x <= 59
 w_constrs[4,7]: x <= 0
 w_constrs[4,8]: x <= 59
 w_constrs[4,9]: x[4,9] - 59 y[4,9] <= 0
 w_constrs[4,10]: x[4,10] - 59 y[4,10] <= 0
 w_constrs[5,1]: x <= 63
 w_constrs[5,2]: x <= 63
 w_constrs[5,3]: x <= 0
 w_constrs[5,4]: x <= 0
 w_constrs[5,5]: x <= 63
 w_constrs[5,6]: x <= 0
 w_constrs[5,7]: x <= 0
 w_constrs[5,8]: x <= 0
 w_constrs[5,9]: x[5,9] - 63 y[5,9] <= 0
 w_constrs[5,10]: x[5,10] - 63 y[5,10] <= 0
Bounds
 Constant = 1
Binaries
 y[1,9] y[2,9] y[3,9] y[4,9] y[5,9] y[1,10] y[2,10] y[3,10] y[4,10] y[5,10]
End
