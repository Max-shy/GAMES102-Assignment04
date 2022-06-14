#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>
#include "../Eigen/Sparse"
#include "../Eigen/Dense"
# define M_PI 3.14159265358979323846
using namespace Ubpa;

//参数化函数
Eigen::VectorXf Update_Uniform_Parameterization(std::vector<Ubpa::pointf2> points);
Eigen::VectorXf Update_Chordal_Parameterization(std::vector<Ubpa::pointf2> points);
Eigen::VectorXf Update_Centripetal_Parameterization(std::vector<Ubpa::pointf2> points);
Eigen::VectorXf Update_Foley_Parameterization(std::vector<Ubpa::pointf2> points);
float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points);

//计算样条曲线
std::vector<Ubpa::pointf2> Draw_Spline(Eigen::VectorXf t, std::vector<Ubpa::pointf2>, float sampling_period = 0.001f);

//绘制函数
void DrawCurve(ImDrawList* draw_list, float origin_x, float origin_y, std::vector<pointf2> draw_points,ImU32 color);

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");
			ImGui::Separator();
			
			ImGui::Separator();
			ImGui::RadioButton("Draw Cubic Spline\t", &data->btn1, 0); ImGui::SameLine();
			ImGui::RadioButton("Edit Cubic Spline\t", &data->btn1, 1);

			if (data->btn1 == 0 || data->btn1 == 1) {
				ImGui::SameLine();ImGui::Checkbox("Uniform", &data->opt_Uniform);
				ImGui::SameLine();ImGui::Checkbox("Chordal", &data->opt_Chordal);
				ImGui::SameLine(); ImGui::Checkbox("Centripetal", &data->opt_Centripetal);
				ImGui::SameLine();ImGui::Checkbox("Foley", &data->opt_Foley);
			}

			ImGui::RadioButton("Draw Bezier Curve\t", &data->btn1, 2); ImGui::SameLine();
			ImGui::RadioButton("Edit Bezier Curve\t", &data->btn1, 3);

			if (data->btn1 == 3) {
				ImGui::SameLine(); ImGui::RadioButton("C0\t",&data->btn2,0);
				ImGui::SameLine(); ImGui::RadioButton("G1\t",&data->btn2,1);
				ImGui::SameLine(); ImGui::RadioButton("C1\t",&data->btn2,2);
			}
			//ImGui::InputInt("Sample Num", &data->SampleNum);
			ImGui::Text("Press [Space] to finish drawing; Press [Z] to print the pos of all type points.");

			// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
			ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
			ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
			if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
			if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
			ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

			// Draw border and background color
			ImGuiIO& io = ImGui::GetIO();
			ImDrawList* draw_list = ImGui::GetWindowDrawList();
			draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
			draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			//Draw Curve
			if (data->btn1 == 0 || data->btn1 == 2) {
				//if choose to draw Spline or Bezier, stop editing the points
				data->editing_points = false;
				data->editing_type_points = false;
				data->editing_control_points = false;

				//create points
				if (is_hovered && !data->adding_line && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					data->points.push_back(mouse_pos_in_canvas);
					data->adding_line = true;
				}
				if (data->adding_line)
				{
					data->points.back() = mouse_pos_in_canvas;
					if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Space))) {//如果按下空格
						if (data->points.size() == 2) {//仅有两点，清空点列
							data->points.clear();
							data->control_points.clear();
						}
						else {
							data->points.resize(data->points.size() - 1);
							if (data->control_points.size() > 2)
								data->control_points.resize(data->control_points.size() - 3);
						}
						data->adding_line = false;
					}
					else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {//继续增加点
						data->points.push_back(mouse_pos_in_canvas);
					}
				}
			}


			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
			{
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line) {
					if (data->points.size() == 2) {
						data->points.clear();
						data->control_points.clear();
					}
					else {
						data->points.resize(data->points.size() - 1);
						if (data->control_points.size() > 2)
							data->control_points.resize(data->control_points.size() - 3);
					}
					data->adding_line = false;
				}

				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) {
					if (data->points.size() == 2) {
						data->points.clear();
						data->control_points.clear();
					}
					else {
						data->points.resize(data->points.size() - 1);
						if (data->control_points.size() > 2)
							data->control_points.resize(data->control_points.size() - 3);
					}
				}
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { 
					data->points.clear(); 
					data->control_points.clear();
				}
				ImGui::EndPopup();

			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid)
			{
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}


			//Draw point and Curve
			if (data->points.size() > 1) {
				std::vector<Ubpa::pointf2> draw_points(data->points);//加入绘图点
				//Draw Spline
				if (data->btn1 == 0 || data->btn1 == 1) {//如果选择绘制/控制样条，计算并绘制样条
					if (data->opt_Uniform) {//均匀参数化
						if (data->points.size() > 2) {
							Eigen::VectorXf t = Update_Uniform_Parameterization(data->points);
							draw_points = Draw_Spline(t,data->points);
						}
						DrawCurve(draw_list, origin.x, origin.y, draw_points, IM_COL32(64, 128, 255, 255));
					}

					// 除Uniform均匀参数化外，其它三种参数化都要求相邻型值点不能重合，否则输出nan/inf或报错，因此拷贝一份型值点集进行处理
					std::vector<pointf2> data_points(data->points);
					std::vector<pointf2>::iterator iter = data_points.begin();
					std::vector<pointf2>::iterator iterEnd = data_points.end();
					while (iter != iterEnd - 1) {
						if ((*iter) == (*(iter + 1))) {
							data_points.erase(iter); // 若当前位置之后存在重复元素，删除当前元素
							break;
						}
						iter++;
					}
					draw_points.assign(data_points.begin(), data_points.end());

					if (data->opt_Chordal) {//距离参数化
						if (data->points.size() > 2) {
							Eigen::VectorXf t = Update_Chordal_Parameterization(data->points);
							draw_points = Draw_Spline(t, data->points);
						}
						DrawCurve(draw_list, origin.x, origin.y, draw_points, IM_COL32(128, 255, 255, 255));
					}
					if (data->opt_Centripetal) {
						if (data->points.size() > 2) {
							Eigen::VectorXf t = Update_Centripetal_Parameterization(data->points);
							draw_points = Draw_Spline(t, data->points);
						}
						DrawCurve(draw_list, origin.x, origin.y, draw_points, IM_COL32(255, 128, 128, 255));
					}
					if (data->opt_Foley) {
						if (data->points.size() > 2) {
							Eigen::VectorXf t = Update_Foley_Parameterization(data->points);
							draw_points = Draw_Spline(t, data->points);
						}
						DrawCurve(draw_list, origin.x, origin.y, draw_points, IM_COL32(255, 64, 64, 255));
					}
				}
				else if (data->btn1 == 2 || data->btn1 == 3) {//如果选择绘制/控制Bezier曲线
					if (data->btn1 == 2 || data->control_points.size() != (data->points.size() * 3 - 2)) {
						//如果每两个点不是由四个控制点控制
						//初始化Bezier曲线的控制点
					}
					//计算bezier曲线
					//绘制Bezier曲线
				}

				//绘制点列
				for (int n = 0; n < data->points.size() - 1; n++) {
					draw_list->AddCircleFilled(ImVec2(data->points[n][0] + origin.x, data->points[n][1] + origin.y), 4.0f, IM_COL32(255, 255, 0, 255));
				}
				if (!data->adding_line)
					draw_list->AddCircleFilled(ImVec2(data->points[data->points.size() - 1][0] + origin.x, data->points[data->points.size() - 1][1] + origin.y), 4.0f, IM_COL32(255, 255, 0, 255));

				//绘制控制点 与 控制线
				if (data->btn1 == 3 && data->editing_points) {
		
				}
			}
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}

//参数化
Eigen::VectorXf Update_Uniform_Parameterization(std::vector<Ubpa::pointf2> points) {
	int n = points.size();
	return Eigen::VectorXf::LinSpaced(n, 0, 1);//[0,1]等距分成n份，等距离参数化
}
Eigen::VectorXf Update_Chordal_Parameterization(std::vector<Ubpa::pointf2> points) {
	int n = points.size();
	Eigen::VectorXf Chordal_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2) {
		Chordal_Result(n - 1) = 1;
		return Chordal_Result;
	}

	for (size_t i = 1; i < n; i++) {
		Chordal_Result[i] = Chordal_Result[i - 1] + (points[i] - points[i - 1]).norm();
	}

	Chordal_Result /= Chordal_Result[n - 1];

	return Chordal_Result;
}
Eigen::VectorXf Update_Centripetal_Parameterization(std::vector<Ubpa::pointf2> points) {
	int n = points.size();
	Eigen::VectorXf Centripetal_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		Centripetal_Result(n - 1) = 1;
		return Centripetal_Result;
	}

	for (size_t i = 1; i < n; i++) {
		Centripetal_Result[i] = Centripetal_Result[i - 1] + std::sqrt((points[i] - points[i - 1]).norm());
	}
	Centripetal_Result /= Centripetal_Result[n - 1];

	return Centripetal_Result;
}
Eigen::VectorXf Update_Foley_Parameterization(std::vector<Ubpa::pointf2> points) {
	int n = points.size();
	Eigen::VectorXf Foley_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		Foley_Result(n - 1) = 1;
		return Foley_Result;
	}

	float d_prev = 0, d_next = 0;
	for (size_t i = 0; i < n - 1; i++)
	{
		float dx = points[i + 1][0] - points[i][0];
		float dy = points[i + 1][1] - points[i][1];
		float chord = sqrt(dx * dx + dy * dy);

		if (i == n - 2) d_next = 0;
		else {
			float dx = points[i + 2][0] - points[i + 1][0];
			float dy = points[i + 2][1] - points[i + 1][1];

			d_next = sqrt(dx * dx + dy * dy);
		}

		float factor = 1.0;
		if (i == 0)
		{
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, points));
			factor = 1 + 1.5 * (theta_next * d_next) / (chord + d_next);
		}
		else if (i == n - 2) {
			float theta = fminf(M_PI / 2, _getAlpha(i, points));
			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord);
		}
		else {
			float theta = fminf(M_PI / 2, _getAlpha(i, points));
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, points));

			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord) + 1.5 * (theta_next * d_next) / (chord + d_next);
		}

		Foley_Result[i + 1] = Foley_Result[i] + chord * factor;

		d_prev = chord;
	}

	Foley_Result /= Foley_Result[n - 1];
	return Foley_Result;
}
float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points) {
	float dx, dy;
	// d_prev
	dx = points[i][0] - points[i - 1][0];
	dy = points[i][1] - points[i - 1][1];
	float d_prev = sqrt(dx * dx + dy * dy);

	// d_next
	dx = points[i + 1][0] - points[i][0];
	dy = points[i + 1][1] - points[i][1];
	float d_next = sqrt(dx * dx + dy * dy);

	// l2
	dx = points[i + 1][0] - points[i - 1][0];
	dy = points[i + 1][1] - points[i - 1][1];
	float l2 = dx * dx + dy * dy;

	float alpha = M_PI - acos((d_prev * d_prev + d_next * d_next - l2 / (2 * d_next * d_prev)));

	return alpha;
}


//曲线绘制
void DrawCurve(ImDrawList* draw_list, float origin_x, float origin_y, std::vector<pointf2> draw_points, ImU32 color) {

	//	col = IM_COL32(255, 128, 128, 255);
	//	col = IM_COL32(255, 64, 64, 255);
	//	col = IM_COL32(0, 255, 255, 255);
	//	col = IM_COL32(254, 67, 101, 255);
	for (int n = 0; n < draw_points.size() - 1; n++) {
		draw_list->AddLine(ImVec2(origin_x + draw_points[n][0], origin_y + draw_points[n][1]), ImVec2(origin_x + draw_points[n + 1][0], origin_y + draw_points[n + 1][1]), color, 2.0f);
	}
}

//计算三次样条曲线(没明白)
std::vector<Ubpa::pointf2> Draw_Spline(Eigen::VectorXf t, std::vector<Ubpa::pointf2> points, float sampling_period) {
	//对于 n + 1 个数据点，相邻两点之间用三次多项式表达，则有 n 段三次多项式
	//用待定系数的方式设第i段向量型多项式函数P(t) = a + bt + ct^2 + dt^3
	std::vector<pointf2> draw_points;
	Eigen::VectorXf M_x = Eigen::VectorXf::Zero(points.size());//对X的M矩阵
	Eigen::VectorXf M_y = Eigen::VectorXf::Zero(points.size());//对Y的M矩阵
	Eigen::VectorXf h(points.size() - 1), b_x(points.size() - 1), b_y(points.size() - 1);//参数向量b
	Eigen::VectorXf u(points.size() - 2), v_x(points.size() - 2), v_y(points.size() - 2), M_mid_x(points.size() - 2), M_mid_y(points.size() - 2);

	//计算相邻两点的矩阵/向量参数
	for (int i = 0; i < points.size() - 1; i++) {
		h(i) = t[i + 1] - t[i];
		b_x(i) = 6.0f * (points[i + 1][0] - points[i][0]) / h(i);
		b_y(i) = 6.0f * (points[i + 1][1] - points[i][1]) / h(i);

		if (i > 0) {
			u(i - 1) = 2.0f * (h(i) + h(i - 1));
			v_x(i - 1) = b_x(i) - b_x(i - 1);
			v_y(i - 1) = b_y(i) - b_y(i - 1);
		}
	}

	//计算Mx=b，中的M矩阵
	if (points.size() == 3) {
		M_mid_x(0) = v_x(0) / u(0);
		M_mid_y(0) = v_y(0) / u(0);
	}
	else if (points.size() == 4) {
		Eigen::Matrix2f A;
		A(0, 0) = u(0);
		A(0, 1) = h(1);
		A(1, 0) = h(1);
		A(1, 1) = u(1);
		M_mid_x = A.colPivHouseholderQr().solve(v_x);//Av_x = M_mid
		M_mid_y = A.colPivHouseholderQr().solve(v_y);//Av_x = M_mid
	}
	else {
		Eigen::SparseMatrix<float> A(points.size() - 2, points.size() - 2);
		A.reserve(Eigen::VectorXi::Constant(points.size() - 2, 3));
		//AM = v_x, AM = v_y, A由u,h组成的对角矩阵，三弯矩
		for (int i = 0; i < points.size() - 2; i++) {
			A.insert(i, i) = u(i);
			if (i > 0)
				A.insert(i, i - 1) = h(i);
			if (i < points.size() - 3)
				A.insert(i, i + 1) = h(i + 1);
		}
		A.makeCompressed();

		//solve the three-moment equations "AM_x=v_x", "AM_y=v_y"
		Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
		solver.compute(A);
		M_mid_x = solver.solve(v_x);
		M_mid_y = solver.solve(v_y);
	}
	for (int i = 0; i < M_mid_x.size(); i++) {
		M_x(i + 1) = M_mid_x(i);
		M_y(i + 1) = M_mid_y(i);
	}
	for (int i = 0; i < points.size() - 1; i++) {
		for (float t1 = t[i]; t1 <= t[i + 1]; t1 += sampling_period) {
			float x = M_x(i) / (6.0f * h(i)) * pow(t[i + 1] - t1, 3) + M_x(i + 1) / (6.0f * h(i)) * pow(t1 - t[i], 3) + (points[i + 1][0] / h(i) - M_x(i + 1) * h(i) / 6.0f) * (t1 - t[i]) + (points[i][0] / h(i) - M_x(i) * h(i) / 6.0f) * (t[i + 1] - t1);
			float y = M_y(i) / (6.0f * h(i)) * pow(t[i + 1] - t1, 3) + M_y(i + 1) / (6.0f * h(i)) * pow(t1 - t[i], 3) + (points[i + 1][1] / h(i) - M_y(i + 1) * h(i) / 6.0f) * (t1 - t[i]) + (points[i][1] / h(i) - M_y(i) * h(i) / 6.0f) * (t[i + 1] - t1);
			
			const pointf2 pos(x, y);
			draw_points.push_back(pos);
		}
	}
	return draw_points;
}


