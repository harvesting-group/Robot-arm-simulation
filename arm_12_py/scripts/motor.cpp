double alpha_arm2 = read_motor_pos(motorz1_id); //- 180;	 // 读取第一个电机位置（回转关节1）
	double beta_arm2 = -read_motor_pos(motorz2_id);			 // 读取第二个电机位置
	double gama_arm2 = -read_motor_pos(motorz3_id);			 // 读取第三个电机位置
	double delta_arm2 = read_motor_pos(motorz4_id);			 // 读取第四个电机位置
	double motor_z5 = read_motor_pos(motorz5_id);          // 读取第五个关节位置
	double z1_arm2 = read_motor_pos(motorx1_id);			 // 读取第五个电机位置 归零侧伺服舵机
	double z2_arm2 = read_motor_pos(motorx2_id);			 // 读取第六个电机位置

	double v_real_alpha_arm2 = read_motor_speed(motorz1_id); // 读取第一个电机速度
	double v_real_beta_arm2 = read_motor_speed(motorz2_id);	 // 读取第二个电机速度
	double v_real_z1_arm2 = read_motor_speed(motorx1_id);	 // 读取第五个电机速度
	double v_real_z2_arm2 = read_motor_speed(motorx2_id);	 // 读取第六个电机速度
	// cout << "zzzzzzz3: " << gama_arm2 << endl;
	geometry_msgs::Pose arm_pos_single;

	float s2 = sin(alpha_arm2 * PI / 180), c2 = cos(alpha_arm2 * PI / 180); // 计算角度的正弦和余弦
	float s3 = sin(beta_arm2 * PI / 180), c3 = cos(beta_arm2 * PI / 180);
	double d44 = d4 * cos(gama_arm2 * PI / 180); // 角度转化为弧度
	double a = z1_arm2;
	//double b = z2_arm2 - z1_arm2 + e3; // 两滑块间距离
	//float Px = a + 30 + (b + e3) / 2 + e2;	   //
	//float Pz = sqrt(d1 * d1 - (b + e3) / 2 * (b + e3) / 2) + h + e1;
  double b = z2_arm2 - z1_arm2 + e3 + e4;
	float Px = a + (b +e4)/ 2 ;	   //
	float Pz = sqrt(d1 * d1 - (b - e2 * 2) / 2 * (b - e2 * 2) / 2) + h + e1;
	float arm2_x = Px + c2 * d2 + c3 * c2 * (d3 + d44) - (d3 + d44) * s2 * s3;
	float arm2_y = Py + c10 * d2 * s2 + c2 * c10 * (d3 + d44) * s3 + c3 * c10 * (d3 + d44) * s2;
	float arm2_z = Pz + d2 * s2 * s10 + c2 * d3 * s3 * s10 + c3 * d3 * s2 * s10 + d4 * sin(gama_arm2 * PI / 180);