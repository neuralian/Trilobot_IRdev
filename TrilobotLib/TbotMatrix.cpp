// TbotMatrix.cpp
// MGP June 2015

#include <TbotMatrix.h>

// constructor, m x n zero matrix
 Matrix::Matrix(unsigned m, unsigned n)
 {  
   nRow = m;
   nCol = n;  
   for(int i=0;i<m;i++)  
     for(int j=0;j<n;j++)  
     dat[i][j]=0.0;  
     
   nIts = _MATRIX_INVERSE_ITERATIONS;
   scale = 0.0;
 }  
 
 // constructor, m x m identity matrix
 Matrix::Matrix(unsigned m) {
       nRow = m;
       nCol = m;
       for (int i=0; i<m; i++) 
          for (int j=0; j<m; j++) 
            dat[i][j] = (i==j)? 1.0:0.0;
     }
	 
	 
	 
void Matrix::assign(int i, int j, float x) { dat[i][j] = x;}
float Matrix::get(int i, int j) { return(dat[i][j]);}


 // addition
 Matrix Matrix :: operator+ (Matrix B) { 
   
  Matrix C(nRow, nCol);  
   
  for(int i=0;i<nRow;i++)  
     for(int j=0;j<nCol;j++)  
       C.dat[i][j]=dat[i][j]+B.dat[i][j];  
   
   return C;  
 }  
   
 // subtraction
 Matrix Matrix :: operator- (Matrix B)  
 {  
  Matrix C(nRow, nCol);  
   
  for(int i=0;i<nRow;i++)  
     for(int j=0;j<nCol;j++)  
       C.dat[i][j]=dat[i][j]-B.dat[i][j];  
    
    return C;  
 }  
 
// unary -
Matrix Matrix :: operator- ()
 {  
  Matrix B(nRow, nCol);  
   
  for(int i=0;i<nRow;i++)  
    for(int j=0;j<nCol;j++)  
       B.dat[i][j]= -dat[i][j];  
   
   return B;  
 } 
   

 
 // multiplication
 Matrix Matrix :: operator* (Matrix B)  
 {  
  Matrix C(nRow, B.nCol);  
  
  if (nCol==B.nRow) {
   
  for(int i=0;i<nRow;i++)   
     for(int j=0;j<B.nCol;j++)   
       for(int k=0;k<nCol;k++)  
         C.dat[i][j]+=dat[i][k]*B.dat[k][j];  
         
  }
  else Serial.println("Matrix * dimension mismatch");
       
   return C;  
 }  
 
  // scalr post-multiplication
 Matrix Matrix :: operator* (float c)  
 {  
  Matrix B(nRow, nCol);  
  
  for(int i=0;i<nRow;i++)   
     for(int j=0;j<nCol;j++)     
         B.dat[i][j]=c*dat[i][j];  

   return B;  
 }  
 
// transpose 
Matrix Matrix :: operator! ()
 {  
  Matrix B(nCol, nRow);  
   
  for (int i = 0; i<nCol; i++)
     for(int j = 0; j<nRow; j++) 
       B.dat[i][j] = dat[j][i];  
   
   return B;  
 } 


// generalized inverse
// X = A/X0 is Newton iteration to A^+ from initial X0
// fixed number of iterations with no termination/accuracy check
// nb X0 must be Matrix(nCol, nRow);
// Xk+1 = Xk(2I - A Xk)
// This map converges to the generalized inverse if the initial 
// guess X0 < alpha*transpose(A) for some small alpha
// So for initial call set X0 = (say) 0.1 transpose(A), 
// for sequential updates use the returned matrix as IC 
// note that the returned Matrix is actually the updated X 
Matrix Matrix :: operator/ (Matrix X)
 {  
   float Y[nRow][nRow];
   float Z[nCol][nRow];


   int rep, i, j, k;
   
   // scale 
   if (scale==0.0) {// needs initializing, find largest entry in dat
     scale = 1.0;
     for (i=0; i<nRow; i++)
       for (j=0; j<nCol; j++) {
	     if (abs(dat[i][j])>scale) scale = abs(dat[i][j]);
		 X.dat[j][i] /=scale;  // scale initial guess
		 }
   }
   else if (scale>1.0) { // this is a continuation, need to rescale X
     for (i=0; i<nRow; i++)
       for (j=0; j<nCol; j++) X.dat[j][i] *= scale;
   }  
   
   // rescale this
   if (scale>1.0) {
     for (i=0; i<nRow; i++)
       for (j=0; j<nCol; j++){
         dat[i][j]/=scale;
		
     }
   }
   

   for (rep=0; rep<nIts; rep++) {
     
	 // Y = 2*eye(nRow) - A*X
     for (i=0; i<nRow; i++) {
       for (j=0; j<nRow; j++) {
          Y[i][j] = (i==j)? 2.0:0.0;
          for (k=0; k<nCol; k++) {
		     Y[i][j] -= dat[i][k]*X.dat[k][j];
		//debug	 Serial.print(X.dat[k][j]); Serial.print(", ");
			 }
		// debug Serial.print(Y[i][j]); Serial.print(", ");
		  }
		 // debug  Serial.println(" ");
       }
	   
	 
	   
	 // Z = X*Y = X*(2*eye(nRow) - A*X)
	 // nb Z is a tmp variable to avoid overwriting X
     for (i=0; i<nCol; i++)
       for (j=0; j<nRow; j++) {
          Z[i][j] = 0.0;
          for (k=0; k<nRow; k++)  Z[i][j] += X.dat[i][k]*Y[k][j];
          X.dat[i][j] = Z[i][j];
      }
	   
	 // // copy the result to X
     // for (i=0; i<nCol; i++)
       // for (j=0; j<nRow; j++) 
         // X.dat[i][j] = Z[i][j];       
   
    }
   
   // rescale this and X
   for (i=0; i<nRow; i++)
    for (j=0; j<nCol; j++) {
         X.dat[j][i] /= scale;  
         dat[i][j] *= scale;
       }     

   
  return(X);
}

// number of iterations for inverse calc
void Matrix ::setnIts(int N) { nIts = N; }
// re-initialize inverse calc
void Matrix ::invertInit(void){ scale=0;}

// Matrix serial print
void Matrix ::SerialPrint(void){
  for (int i=0; i<nRow; i++) {
    for (int j=0; j<nCol; j++) {
      Serial.print(dat[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  Serial.println("");
} 
