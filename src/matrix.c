#include "matrix.h"

Matrix *matrix_alloc(int row_size, int col_size)
{
	int j;
	Matrix *new_matrix = malloc(sizeof(Matrix)); 

	//Allocating memory for the new matrix structure
	new_matrix->row_size = row_size;
	new_matrix->col_size = col_size;
	new_matrix->matrix_entry = malloc( new_matrix->row_size *sizeof(float *));
	for(j = 0 ; j < new_matrix->row_size ; j++)
	{
		new_matrix->matrix_entry[j] = malloc( new_matrix->col_size*sizeof(float) );
	}
	
	return new_matrix;
}

/*Copies Matrix1 into matrix2 */
void matrix_copy(Matrix *matrix1, Matrix *matrix2)
{
	int i, j;
	for (i = 0; i < matrix1->row_size; i += 1)
	{
		for (j = 0; j < matrix1->col_size; j += 1)
		{
			matrix2->matrix_entry[i][j] = matrix1->matrix_entry[i][j];
		}
	}
}


Matrix *matrix_multiply(const Matrix *matrix1, const Matrix *matrix2)
{
	int i, j,k, sum;
	if (matrix1->col_size != matrix2->row_size)
	  {
	    //terminate("ERROR: The number columns of matrix1  != number of rows in matrix2!");
	  }
	Matrix *result = matrix_alloc( matrix1->row_size,matrix2->col_size);
	for (i = 0; i < matrix1->row_size; i += 1)
	{
		for (k = 0; k < matrix2->col_size; k += 1)
		{
			sum = 0;
			
			for (j = 0; j < matrix1->col_size; j += 1)
			{
				sum += matrix1->matrix_entry[i][j] * matrix2->matrix_entry[j][k];
			}
			
			result->matrix_entry[i][k] = sum;
		}
	}
       return result;
}

void matrix_free( Matrix *matrix)
{
	int j;
	for(j = 0 ; j < matrix->row_size ; j++)
	  {
		free(matrix->matrix_entry[j]); 
	  }
	free(matrix->matrix_entry);
	free(matrix);
}



/*Function which divides all row entries by the value of a the diagonal */
void row_divide(Matrix *matrix, int pivot)
{
    int j;
    float 	divisor = matrix->matrix_entry[pivot][pivot], 
              result;

    for(j = pivot; j < matrix->col_size; j++)
    {
              result = (matrix->matrix_entry[pivot][j]  /  divisor);
              matrix->matrix_entry[pivot][j] = result;
    }

}

 /*Function to carry out row operations*/
void row_operation(Matrix *multiplier_matrix,Matrix *matrix, int pivot, int row_index)
{
    int j;
    float multiplier = (matrix->matrix_entry[row_index][pivot] / matrix->matrix_entry[pivot][pivot]);
    //Loop which checks if matrix is provided to store the multiplier
    if(multiplier_matrix != NULL)
      {
	multiplier_matrix ->matrix_entry[row_index][pivot] = multiplier;
      }

    
    for(j=0; j < matrix->col_size; j++)
    {
	    matrix->matrix_entry[row_index][j] -=  multiplier * matrix->matrix_entry[pivot][j];
    }
}

void matrix_row_reduce( Matrix *matrix, int zero_control )
{
    int pivot, row_index;
    float multiplier;
    for( pivot = 0; pivot < matrix->row_size ; pivot++)
    {
         
      error_zeros(matrix, zero_control); //Function checks if there are too many zeros in a single row
	    if(	(matrix->matrix_entry[pivot][pivot] != 1) && (matrix->matrix_entry[pivot][pivot] != 0)	)
	    {
		row_divide(matrix, pivot);
	    }

	    for (row_index = pivot+1; row_index < matrix->row_size; row_index++)
	    {
		    if (matrix->matrix_entry[pivot][pivot] != 0)
		    {
		      row_operation(NULL,matrix, pivot, row_index);
		    }
	    }

		for(row_index = pivot-1; row_index >=0; row_index --)
		{
			if (matrix->matrix_entry[pivot][pivot] != 0)
			{
			  row_operation(NULL,matrix, pivot, row_index);
			}
		}
	}
}

void matrix_subtract(Matrix *result, Matrix *matrix1, Matrix *matrix2)
{
	int i, j;



 for(i = 0; i < matrix1->row_size; i += 1)
	{
		for (j = 0; j < matrix1->col_size; j += 1)
		{
			result->matrix_entry[i][j] = matrix1->matrix_entry[i][j] - matrix2->matrix_entry[i][j];
		}
	}
}

void matrix_add(Matrix *result, Matrix *matrix1, Matrix *matrix2)
{
	int i,j;

	for (i = 0; i < matrix1->row_size; i += 1)
	{
		for (j = 0; j < matrix1->col_size; j += 1)
		{
			result->matrix_entry[i][j] = matrix1->matrix_entry[i][j] + matrix2->matrix_entry[i][j];
		}
	}
}

Matrix *matrix_transpose(Matrix *m) {
	Matrix *t = matrix_alloc(m->col_size, m->row_size);
	int i,j;
	for(i=0;i<m->row_size;i++) {
		for(j=0;j<m->col_size;j++) {
			t->matrix_entry[j][i]=m->matrix_entry[i][j];
		}
	}
	return t;
}

void matrix_invert(Matrix *inverse_matrix)
{
  int j,k;
  /*Temporal matrix used in this function */
  Matrix *temp_matrix = matrix_alloc(inverse_matrix->row_size, inverse_matrix->col_size *2); 

  matrix_copy(inverse_matrix, temp_matrix);

 /* Adding an identity matrix at the end of the temporal matrix */
  for(j = 0; j< temp_matrix->row_size; j++)
    {
      for(k = 3; k < temp_matrix->col_size; k++)
      {
	if( j+3  == k)
	  {
	    temp_matrix->matrix_entry[j][k] = 1; 
	  }
	else
	  {
	    temp_matrix->matrix_entry[j][k] = 0;
	  }
      }
    }

  matrix_row_reduce(temp_matrix, temp_matrix->row_size);

  /* Copying the inverse matrix from the temp_matrix to the  invse_matrix */
  for(j = 0; j< temp_matrix->row_size; j++)
    {
      for(k = 3; k < temp_matrix->col_size; k++)
      {
	inverse_matrix->matrix_entry[j][k-3] = temp_matrix->matrix_entry[j][k];
      }
    }
  
  matrix_free(temp_matrix);
}

/*int matrix_equal_size( Matrix *matrix1, Matrix *matrix2)
{

  return (matrix1->row_size == matrix2->row_size && \
	      matrix1->col_size == matrix2->col_size);
}*/

/*
  This function checks if there is a line containing too many zero's and it exits
  if such a line is found
*/
void error_zeros( Matrix *matrix, int control_index)
{
      int i,j,count;
      for(i=0; i<matrix->row_size; i++)
      {
	    count=0;
	    for(j = 0;  j < matrix->col_size; j++)
	    {
	      if( matrix->matrix_entry[i][j] == 0)
	      {
		count++;     
	      }
	      else
	      {
		return;
	      }
	      if(count == control_index)
	      {
		//fprintf(stdout,"\nProcess fail because row %d contains %d  zeros\n",i+1,control_index);
		//matrix_print(matrix);
		//exit(1);
	      }
	    }
	  }
}  


/*void terminate (char * string)
{
  //fprintf(stdout,"\n%s\n",string);
  //fprintf(stdout,"The program is exiting now. . . .\n\n");
  //exit(-1);
}*/
