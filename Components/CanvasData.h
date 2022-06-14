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
	bool editing_points{ false };//�༭��
	bool editing_type_points{ false };
	bool editing_control_points{ false };//�༭���Ƶ�

	//���߲�����
	bool opt_Uniform{ false };//���Ȳ�����
	bool opt_Chordal{ false };//Chordal������
	bool opt_Centripetal{ false };//ƽ�����Ĳ�����
	bool opt_Foley{ false };//Foley������

	//�����л�״̬
	int btn1{ 0 };//����1:ȷ���������ͣ�Bezier or Spline���������draw or Edit��
	int btn2{ 0 };//����2:ȷ�����������Եȼ���C0,C1,G1


};

#include "details/CanvasData_AutoRefl.inl"
