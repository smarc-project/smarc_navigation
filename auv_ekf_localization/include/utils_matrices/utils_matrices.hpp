/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UTILS_MATRICES_HPP
#define UTILS_MATRICES_HPP

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <boost/scoped_ptr.hpp>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>

namespace matrices{

    template<typename T>
    T matDeterminant(const boost::numeric::ublas::matrix<T>& mat_A){
        using namespace boost::numeric::ublas;
        matrix<T> mLu(mat_A);
        permutation_matrix<std::size_t> pivots(mat_A.size1());

        auto isSingular = lu_factorize(mLu, pivots);
        if (isSingular){
            return static_cast<T>(0);
        }

        T det = static_cast<T>(1);
        for (std::size_t i = 0; i < pivots.size(); ++i){
            if (pivots(i) != i){
                det *= static_cast<T>(-1);
            }
            det *= mLu(i, i);
        }
        return det;
    }

    template<typename T>
    bool InvertMatrix (const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse) {
        using namespace boost::numeric::ublas;
        matrix<T> A(input);
        // Perform LU-factorization
        permutation_matrix<std::size_t> pm(A.size1());
        int res = lu_factorize(A,pm);
        if( res != 0 )
            return false;
        inverse.assign(identity_matrix<T>(A.size1()));
        lu_substitute(A, pm, inverse);
        return true;
    }

    template<typename T>
    boost::numeric::ublas::matrix<T> Cholesky(const boost::numeric::ublas::matrix<T>& mat_A){
        // TODO_NACHO: check for matrix conditions to use cholesky
        int n = mat_A.size1();
        boost::numeric::ublas::matrix<T> chol_triang(n, n);
        for(unsigned int i=0; i< mat_A.size1(); i++){
            for(unsigned int j=0; j< i + 1; j++){
                double s = 0;
                for(unsigned int k = 0; k<j; k++){
                    s += chol_triang(i * n + k) * chol_triang(j * n + k);
                }
                chol_triang(i * n + j) = (i = j)?
                            std::sqrt(mat_A(i * n + i) - s):
                            (1.0 / chol_triang(j * n + j) * (mat_A(i * n + j) - s));
            }
        }
        return chol_triang;
    }

    template<typename T>
    boost::numeric::ublas::matrix<T> matTriangDeterminant(const boost::numeric::ublas::matrix<T>& mat_A){
        int n = mat_A.size1();
        T det;
        for(unsigned int i=0; i< mat_A.size1(); i++){
            for(unsigned int j=0; j< mat_A.size2(); j++){
                det *= (i == j)? mat_A(i,j): 1;
            }
        }
        return det;
    }
}

#endif // UTILS_MATRICES_HPP
