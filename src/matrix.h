#include <stdlib.h>

struct _Matrix
{
  /** The number  of rows in the matrix */
  int row_size;

  /** The number of columns in the matrix */
  int col_size;
  
  /** The individual entries in the matrix */
  float **matrix_entry;
};

typedef struct _Matrix Matrix;

/**
 * Allocates memory for a new matrix
 * 
 * @param row_size       The number of rows in the matrix
 * @param col_size         The number of columns in the matrix
 * @return                      The location of the memory block that was 
 *                                       allocated to hold the matrix
 */

Matrix *matrix_alloc(int row_size, int col_size);

/**
 * Copies the content of one matrix1 into matrix2
 *
 * @param matrix1       Pointer  the matrix to be copied
 * @param matrix2       Pointer to the matrix to which the other is to
 *                                   be copied into
 */

void matrix_copy(Matrix *matrix1, Matrix *matrix2);

/**
 * Multiplies two matrices
 * 
 * @param matrix1     Pointer to the first matrix for the multiplication
 * @param matrix2     The second matrix for the multiplication
 * @return                  The location of a matrix which holds the result
 *                                of the multiplication
 */
Matrix *matrix_multiply(const Matrix *matrix1, const Matrix *matrix2);

/**
 * Free an entire matrix
 * 
 * @param matrix       The matrix to free.
 */
void matrix_free(Matrix *matrix);


/**
 * Divides an entire row of a matrix by a value of the pivot position
 * 
 * @param matrix       The matrix whose row is to be divided
 * @param pivot          The pivot position of the matrix to do the division
 */
void row_divide(Matrix *matrix, int pivot);

/**
 * Row operations on the matrix
 *
 * @param multiplier_matrix    A matrix to store the various multipliers used
 *                                             in row reduction
 * @param matrix          A matrix on which to carry out the row
 *                                  operations
 * @param pivot            The pivot position of the matrix to use
 * @param  row_index   The row number on which to carry out row operations
 */

void row_operation(Matrix *multiplier_matrix, Matrix *matrix, int pivot, int row_index);

/**
 * Row echelon reduction a matrix
 *
 * @param matrix     The matrix on which to carry out row reduction
 * @param zero_control      Maximum amount of zeros that can be found on a
 *                                        row
 */

void matrix_row_reduce( Matrix *matrix, int zero_control);

/**
 * Subtracts one matrix from another
 * 
 * @param result         A  matrix to hold the result of the subtraction
 * @param matrix1      The matrix to subtract from
 * @param matrix2       The matrix to be subtracted from another
 */

void matrix_subtract(Matrix *result, Matrix *matrix1, Matrix *matrix2);

/**
 * Adds one matrix to another
 * 
 * @param result        A  matrix to hold the result of the addition
 * @param matrix1     The first matrix for the addition
 * @param matrix2     The second matrix for the addition
*/

void matrix_add(Matrix *result, Matrix *matrix1, Matrix *matrix2);

/**
 * Perform the inverse of a matrix
 *
 * @param inverse_matrix The matrix which is to be inverted
 */
void matrix_invert(Matrix *inverse_matrix);

Matrix * matrix_transpose(Matrix *m);

/**
 * Checks  if there are too many zeros in a single line
 *
 * @param matrix The matrix which is to be checked
 * @param control_index    The maximum amount of zero's that can be contained
 *                                         in a single row
 */
void error_zeros( Matrix *matrix, int control_index);
