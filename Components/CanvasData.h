#pragma once

#include <UGM/UGM.h>

struct CanvasData {
	std::vector<Ubpa::pointf2> points;//点序列
	std::vector<Ubpa::pointf2> control_points;//控制点
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	//编辑状态开关
	bool editing_points{ false };//编辑点开关
	bool editing_type_points{ false };//编辑 编辑点开关
	bool editing_control_points{ false };//操作编辑点的控制点开关

	int type_point_index{ 0 };//编辑点下标
	int control_point_index{ 0 };//控制点下标
	Ubpa::pointf2 selected_type_point;//当前选择的编辑点
	Ubpa::pointf2 selected_ctrl_point1;//当前编辑点的控制点一
	Ubpa::pointf2 selected_ctrl_point2;//当前编辑点的控制点二

	//曲线参数化
	bool opt_Uniform{ false };//均匀参数化
	bool opt_Chordal{ false };//Chordal参数化
	bool opt_Centripetal{ false };//平方中心参数化
	bool opt_Foley{true};//Foley参数化

	//按键切换状态
	int btn1{ 0 };//按键1:确定曲线类型（Bezier or Spline）与操作（draw or Edit）
	int btn2{ 0 };//按键2:确定曲线连续性等级，C0,C1,G1


};

#include "details/CanvasData_AutoRefl.inl"
