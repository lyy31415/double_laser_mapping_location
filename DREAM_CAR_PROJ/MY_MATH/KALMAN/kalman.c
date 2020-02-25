#include "kalman.h"

//float noise_process_omega=0.001f;
//float noise_measurement_omega=0.31f;
float noise_process_omega=0.1f;
float noise_measurement_omega=0.31f;


float A_data[4];
float B_data[2];

float process_noise_covariance_matrix_data[4];	//ִ������Э�������
float measurement_noise_covariance_matrix_data[4];	//��������Э�������
float gain_matrix_data[4];	//����ϵ������

arm_matrix_instance_f32 state_variable_matrix;	//�������˲����е�״̬��
arm_matrix_instance_f32 vcovariance_matrix;	//�������˲����е�Э����

arm_matrix_instance_f32 A_matrix;	//�������˲����е�A
arm_matrix_instance_f32 B_matrix;	//�������˲����е�B

arm_matrix_instance_f32 process_noise_covariance_matrix;	//ִ������Э����
arm_matrix_instance_f32 measurement_noise_covariance_matrix;	//��������Э����

arm_matrix_instance_f32 gain_matrix;	//�������˲����е�����K

float state_variable_data[2];	//״̬����������
float vcovariance_data[4];	//Э������������
float process_data;
float measurement_data[2];

arm_matrix_instance_f32 process_matrix;
arm_matrix_instance_f32 measurement_matrix;

float matrix_data_temp1[10];

static float *matrix_data_temp = matrix_data_temp1;


void kalman_Init_Data(void)
{
	u8 i;
	
	//������Ŷ�
	for (i = 0; i < 4; i++)
	{
		vcovariance_data[i] = 0.0f;
	}
	//���״̬��
	state_variable_data[0] = state_variable_data[1] = 0.0f;
}


void kalman_Init(void)
{
	A_data[0] = 0.0f;
	A_data[1] = 0.0f;
	A_data[2] = 0.0f;
	A_data[3] = -1.0f;
	B_data[1] = 2.0f;
	
	
	arm_mat_init_f32(&process_matrix, 1, 1, &process_data);
	
	arm_mat_init_f32(&state_variable_matrix, 2, 1, state_variable_data);
	arm_mat_init_f32(&vcovariance_matrix, 2, 2, vcovariance_data);

	arm_mat_init_f32(&A_matrix, 2, 2, A_data);
	arm_mat_init_f32(&B_matrix, 2, 1, B_data);

	arm_mat_init_f32(&process_noise_covariance_matrix, 2, 2, process_noise_covariance_matrix_data);
	arm_mat_init_f32(&measurement_noise_covariance_matrix, 2, 2, measurement_noise_covariance_matrix_data);

	arm_mat_init_f32(&gain_matrix, 2, 2, gain_matrix_data);

	kalman_Init_Data();
}


//************************************
// Method:    Set_Noise
// Returns:   void
// Parameter: float noise �����������ڴ˴�Ϊʱ��
// Description: �����˲�����ִ�С���������
//************************************
void kalman_Set_Noise(float noise)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	
	float noise_temp = noise_measurement_omega*noise*noise/4;

	arm_matrix_instance_f32 noise_matrix;
  B_data[0] = noise;
	arm_mat_init_f32(&noise_matrix, 1, 1, &noise_process_omega);

	
	//��ʼ��matrix_temp1,���ڴ��B^T
	arm_mat_init_f32(&matrix_temp1, B_matrix.numCols, B_matrix.numRows, matrix_data_temp);
	arm_mat_trans_f32(&B_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��B*noise
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, noise_matrix.numCols, matrix_data_temp);
	arm_mat_mult_f32(&B_matrix, &noise_matrix, &matrix_temp2);
	//matrix_temp2.pData[0] = B_matrix.pData[0] * noise_process_omega;
	//matrix_temp2.pData[1] = B_matrix.pData[1] * noise_process_omega;

	//����ִ������ B*noise*B^T
	arm_mat_mult_f32(&matrix_temp2, &matrix_temp1, &process_noise_covariance_matrix);

	

	//���²�������
	measurement_noise_covariance_matrix_data[0] = noise_temp;
	measurement_noise_covariance_matrix_data[1] = noise_measurement_omega*noise/2;
	measurement_noise_covariance_matrix_data[2] = noise_measurement_omega*noise/2;
	measurement_noise_covariance_matrix_data[3] = noise_measurement_omega;
}

//************************************
// Method:    Kalman_Filter
// Returns:   arm_matrix_instance_f32 &
// Parameter: const arm_matrix_instance_f32 & input ִ����
// Parameter: const arm_matrix_instance_f32 & measurement ������
// Description: ʹ�ÿ������˲��������µ�״̬��
//************************************
arm_matrix_instance_f32 Kalman_Filter( arm_matrix_instance_f32 input,  arm_matrix_instance_f32 measurement)
{
	Forecast_State_Variable_Process_Model(input);	//Ԥ��״̬��
	Forecast_Covariance();	//Ԥ��Э�������	
	Cal_Kalman_Gain();	//���㿨�����������	
	Update_State_Variable(measurement);	//����״̬��
	Update_Covariance();	//����Э�������
	
	return state_variable_matrix;
}


//************************************
// Method:    Forecast_State_Variable_Process_Model
// Returns:   void
// Parameter: const arm_matrix_instance_f32 & input
// Description: Ԥ��״̬��
//************************************
void Forecast_State_Variable_Process_Model(const arm_matrix_instance_f32  input)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;

	//��ʼ��matrix_temp1,���ڴ��A^state
	arm_mat_init_f32(&matrix_temp1, A_matrix.numRows, state_variable_matrix.numCols, matrix_data_temp);
	arm_mat_mult_f32(&A_matrix, &state_variable_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��B*input
	arm_mat_init_f32(&matrix_temp2, B_matrix.numRows, input.numCols, matrix_data_temp);
	arm_mat_mult_f32(&B_matrix, &input, &matrix_temp2);

	//Ԥ��״̬��
	arm_mat_add_f32(&matrix_temp1, &matrix_temp2, &state_variable_matrix);
}

//************************************
// Method:    Forecast_Covariance
// Returns:   void
// Parameter: void
// Description: Ԥ��Э�������
//************************************
void Forecast_Covariance(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2, matrix_temp3;

	//��ʼ��matrix_temp1,���ڴ��A^T
	arm_mat_init_f32(&matrix_temp1, A_matrix.numCols, A_matrix.numRows, matrix_data_temp);
	arm_mat_trans_f32(&A_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��A*vcovariance
	arm_mat_init_f32(&matrix_temp2, A_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	arm_mat_mult_f32(&A_matrix, &vcovariance_matrix, &matrix_temp2);

	//��ʼ��matrix_temp3,���ڴ��A*vcovariance*A^T
	arm_mat_init_f32(&matrix_temp3, matrix_temp2.numRows, matrix_temp1.numCols, matrix_data_temp);
	arm_mat_mult_f32(&matrix_temp2, &matrix_temp1, &matrix_temp3);

	//Ԥ��Э����
	arm_mat_add_f32(&matrix_temp3, &process_noise_covariance_matrix, &vcovariance_matrix);
}

//************************************
// Method:    Cal_Kalman_Gain
// Returns:   void
// Parameter: void
// Description: ���㿨�����������
//************************************
void Cal_Kalman_Gain(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;

	//��ʼ��matrix_temp1,���ڴ��vcovariance+measurement_noise_covariance
	arm_mat_init_f32(&matrix_temp1, vcovariance_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	arm_mat_add_f32(&vcovariance_matrix, &measurement_noise_covariance_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��(vcovariance+measurement_noise_covariance)^(-1)
	arm_mat_init_f32(&matrix_temp2, matrix_temp1.numCols, matrix_temp1.numRows, matrix_data_temp);
	arm_mat_inverse_f32(&matrix_temp1, &matrix_temp2);

	//���㿨�����������
	arm_mat_mult_f32(&vcovariance_matrix, &matrix_temp2, &gain_matrix);
}

//************************************
// Method:    Update_State_Variable
// Returns:   void
// Parameter: const arm_matrix_instance_f32 & measurement
// Description: ����״̬��
//************************************
void Update_State_Variable(arm_matrix_instance_f32 measurement)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2, matrix_temp3;
	
	//�����µ�״̬��
	int data_size;
	float *source_data = state_variable_matrix.pData;
	float *pdata;
	u8 i;
	

	//��ʼ��matrix_temp1,���ڴ��measurement-state
	arm_mat_init_f32(&matrix_temp1, measurement.numRows, measurement.numCols, matrix_data_temp);
	arm_mat_sub_f32(&measurement, &state_variable_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��gain*(measurement-state)
	arm_mat_init_f32(&matrix_temp2, gain_matrix.numRows, matrix_temp1.numCols, matrix_data_temp);
	arm_mat_mult_f32(&gain_matrix, &matrix_temp1, &matrix_temp2);

	//��ʼ��matrix_temp3,���ڴ��state+gain*(measurement-state)
	arm_mat_init_f32(&matrix_temp3, state_variable_matrix.numRows, state_variable_matrix.numCols, matrix_data_temp);
	arm_mat_add_f32(&state_variable_matrix, &matrix_temp2, &matrix_temp3);

	
	data_size = matrix_temp3.numRows*matrix_temp3.numCols;
	pdata = matrix_temp3.pData;
	
	for (i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}
}

void Update_Covariance(void)
{
	arm_matrix_instance_f32 matrix_temp1, matrix_temp2;
	u8 i;
	
		//�����µ�Э����
	int data_size;
	float *source_data;
	float *pdata;
	

	//��ʼ��matrix_temp1,���ڴ��gain*vcovariance
	arm_mat_init_f32(&matrix_temp1, gain_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	arm_mat_mult_f32(&gain_matrix, &vcovariance_matrix, &matrix_temp1);

	//��ʼ��matrix_temp2,���ڴ��vcovariance-gain*vcovariance
	arm_mat_init_f32(&matrix_temp2, vcovariance_matrix.numRows, vcovariance_matrix.numCols, matrix_data_temp);
	arm_mat_sub_f32(&vcovariance_matrix, &matrix_temp1, &matrix_temp2);

  data_size = matrix_temp2.numRows*matrix_temp2.numCols;
	source_data = vcovariance_matrix.pData;
	pdata = matrix_temp2.pData;

	for (i = 0; i < data_size; i++)
	{
		*source_data = *pdata;
		source_data++;
		pdata++;
	}
}







