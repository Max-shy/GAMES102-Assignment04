#pragma once

#include <UGM/UGM.h>

struct CanvasData {
	std::vector<Ubpa::pointf2> points;//������
	std::vector<Ubpa::pointf2> control_points;//���Ƶ�
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	//�༭״̬����
	bool editing_points{ false };//�༭�㿪��
	bool editing_type_points{ false };//�༭ �༭�㿪��
	bool editing_control_points{ false };//�����༭��Ŀ��Ƶ㿪��

	int type_point_index{ 0 };//�༭���±�
	int control_point_index{ 0 };//���Ƶ��±�
	Ubpa::pointf2 selected_type_point;//��ǰѡ��ı༭��
	Ubpa::pointf2 selected_ctrl_point1;//��ǰ�༭��Ŀ��Ƶ�һ
	Ubpa::pointf2 selected_ctrl_point2;//��ǰ�༭��Ŀ��Ƶ��

	//���߲�����
	bool opt_Uniform{ false };//���Ȳ�����
	bool opt_Chordal{ false };//Chordal������
	bool opt_Centripetal{ false };//ƽ�����Ĳ�����
	bool opt_Foley{true};//Foley������

	//�����л�״̬
	int btn1{ 0 };//����1:ȷ���������ͣ�Bezier or Spline���������draw or Edit��
	int btn2{ 0 };//����2:ȷ�����������Եȼ���C0,C1,G1


};

#include "details/CanvasData_AutoRefl.inl"
