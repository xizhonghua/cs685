## GMU CS 685 Autonoumos Robotics 
#### Instrctuor: Prof. Jana Kosecka 
Course website: [http://cs.gmu.edu/~kosecka/cs685/](http://cs.gmu.edu/~kosecka/cs685/)

### Course Project: Motion Planning for Differential Wheeled Robot with Dynamic Constraints
#### Search for best controls

| k_rho | k_alpha | k_beta|
|:----:|:------:|:------:|
| <img src="/mp/k_rho_3d.png" width="200px"/> | <img src="/mp/k_alpha_3d.png" width="200px"/> | <img src="/mp/k_beta_3d.png" width="200px"/> |
| <img src="/mp/k_rho_proj.png" width="200px"/> | <img src="/mp/k_alpha_proj.png" width="200px"/> | <img src="/mp/k_beta_proj.png" width="200px"/> |

#### Learn an optimal-path distance metric
| Training | Predicted | SE2 |
|:----:|:------:|:------:|
| <img src="/mp/svm_train_3d.png" width="200px"/> | <img src="/mp/svm_test_3d.png" width="200px"/> | <img src="/mp/se2_3d.png" width="200px"/> |
| <img src="/mp/svm_train_proj.png" width="200px"/> | <img src="/mp/svm_test_proj.png" width="200px"/> | <img src="/mp/se2_proj.png" width="200px"/> |

### Result
| Path length | Path time|
|:----:|:------:|
|<img src="/mp/path_length.png" width="300px"/> | <img src="/mp/path_time.png" width="300px"/> |


### Code
Implemented in C++: [mp](/mp)
