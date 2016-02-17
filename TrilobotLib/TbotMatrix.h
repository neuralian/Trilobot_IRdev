// TbotMatrix.h
// Trilobot/Arduino barebones Matrix class header
//  
// inverse is generalized inverse A+
// X = A/X0 is Newton iteration to A+ from initial X0
// fixed number of iterations with no termination/accuracy check
// nb X0 must have dimensions nCol x nRow
// Xk+1 = Xk(2I - A Xk)
// This map converges to the generalized inverse if the initial 
// guess X0 < alpha*transpose(A) for some small alpha
// So for initial call set X0 = (say) 0.1 transpose(A), 
// note that the returned Matrix is X iterated towards the inverse,
// so for sequential updates use the returned matrix as IC 
// without re-initializing (ie calling A.invertInit());
// MGP May 2015

 #include "Arduino.h"
 
#ifndef TbotMatrix_h
#define TbotMatrix_h


 #define _MAX_MATRIX_SIZE 5  
 #define _MATRIX_INVERSE_ITERATIONS 16   
 
  // // utility to check  how much SRAM is available
 // int freeRam () 
// {
  // extern int __heap_start, *__brkval; 
  // int v; 
  // return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
// }
  
   


 class Matrix  
 {  
   private:
   
      int nIts; // number of iterations for inverse
      float dat[_MAX_MATRIX_SIZE][_MAX_MATRIX_SIZE];  
    
    public:  
      unsigned nRow,nCol;
      float scale; // scale factor for iterative matrix inversion
                   // reset to 0 to re-initialize the iteration
                   // (using invertInit())

     Matrix(unsigned m, unsigned n);// create m x n matrix filled with zeros
  
     ~Matrix()  {  delete dat;  }  
     
     // mxm identity matrix
     Matrix(unsigned m);
	 
	 // A[i][j] = x
	 void assign(int i, int j, float x);
	 
	 // x = A[i][j]
	 float get(int i, int j);
     
	 // number of iterations for matrix inversion
     void setnIts(int N);
	 // re-initialize matrix inversion iteration
     void invertInit(void);
      
     // serial print matrix
     void SerialPrint(void);
     
     Matrix operator +(Matrix B);  // add
     Matrix operator -(Matrix B);  // subtract
     Matrix operator *(Matrix B);  // multiply
     Matrix operator *(float c);   // scalar post multiply
     Matrix operator /(Matrix B);  // inverse (Newton iteration from initial guess B)
     Matrix operator -();          // unary -
     Matrix operator !();          // transpose
   
 };

#endif 
 