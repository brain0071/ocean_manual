Eigen::Matrix<double, 6, 8> allo_matrix;
    allo_matrix << 0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736,  // x
                   -0.57736, -0.57736, 0.57736, 0.57736, 0.57736, 0.57736, -0.57736, -0.57736,  // y
                   0.57736, -0.57736, -0.57736, 0.57736, -0.57736, 0.57736, 0.57736, -0.57736,  // z
                   0.199105, 0.199105, 0.199105, 0.199105, -0.199105, -0.199105, -0.199105, -0.199105, // r
                   -0.217233, 0.217233, 0.217233, -0.217233, -0.217233, 0.217233, 0.217233, -0.217233, // p
                   -0.210211, -0.210211, 0.210211, 0.210211, -0.210211, -0.210211, 0.210211, 0.210211; // y
    
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 8>> svd(allo_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double, 8, 6> allo_matrix_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();