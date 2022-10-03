using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity 速度
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity 角速度
	
	float mass;									// mass 质量
	Matrix4x4 I_ref;							// reference  inertia 转动惯量

	float linear_decay	= 0.999f;				// for velocity decay 速度衰减
	float angular_decay	= 0.98f;				//角速度衰减
	float restitution 	= 0.5f;                 // for collision 
	Vector3 grivity = new Vector3(0, -9.8f, 0);//重力
	//向上是y轴的正方向
	//所以这里的g是y的负方向，也就是向下是没有问题的


	// Use this for initialization，这个start是unity自己写的，所以不用自己再调用
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;//取得mesh，具体也不用管
		Vector3[] vertices = mesh.vertices;//取得一个顶点的数组

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) //构建一个矩阵
		{
			mass += m;//调整质量，这里是有多少个网格就有多少的质量
			float diag=m*vertices[i].sqrMagnitude;//质量*向量的平方长度
			//计算转动惯量
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;

	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		//向量a的叉乘矩阵
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;

	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	//在这个函数中更新v和w，因为碰撞到了一个平面<p,n>上
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//看了一下别人的代码，没有把旋转加进去
		Matrix4x4 rotation_matrix = Matrix4x4.Rotate(transform.rotation);//取得旋转矩阵
		//旋转之后的惯性张量
		Matrix4x4 I_rot = rotation_matrix * I_ref * Matrix4x4.Transpose(rotation_matrix);
		//旋转之后的惯性张量的逆
		Matrix4x4 I_inverse = Matrix4x4.Inverse(I_rot);
		//1/m的矩阵
		Matrix4x4 mass_matrix = new Matrix4x4(new Vector4(1.0f/mass,0,0,0),new Vector4(0,1.0f/mass,0,0),new Vector4(0,0,1.0f/mass,0),new Vector4(0,0,0,1.0f/mass) ) ;
		//calculate the position and the velocity of every mesh vertex
		Vector3 temp_position=transform.position;//取得中心点的位置
		Mesh mesh = GetComponent<MeshFilter>().mesh;//取得mesh，具体也不用管
		Vector3[] vertices = mesh.vertices;//取得一个顶点的数组
		//用于计算平均位置的向量
		Vector3 total_positon = new Vector3(0, 0, 0);//初始化为0;
		int counter = 0;//计数器，每次update都要置0
		foreach (Vector3 vt in vertices)
        {

			Vector3 Ri = rotation_matrix.MultiplyVector(vt);
			Vector3 Xi = Ri + temp_position;
			Vector3 vi;//vi是判断该点的速度的中间变量
			if(judgeCollision(Xi,P,N))//如果这个点以及碰撞了
            {
				vi = this.v + Vector3.Cross(w, Ri);//这里之前的写法错了，角速度的线速度还是不用考虑实际的坐标的
				if(Vector3.Dot(vi,N)<0.0f)//如果还是朝着距离函数负的方向
                {
					counter++;
					total_positon +=Ri;//把这个点的坐标算进总的坐标位置
                }
            }
        }
		if(counter==0)//如果还没碰撞,这里是处理碰撞的点数量为0的情况
        {
			return;
        }
		total_positon = total_positon / counter;//求出平均碰撞点
		//total_positon = rotation_matrix.MultiplyVector(total_positon);
		Vector3 Vni, Vti, Vni_new, Vti_new,Vi,Vi_new,j;
		float a;
		Vi = this.v + Vector3.Cross(w, total_positon);//速度
		Vni = (Vector3.Dot(Vi, N)) * N;
		Vti = Vi - Vni;//切向速度
		a = System.Math.Max(1.0f - this.restitution * (1.0f + this.restitution) * Vni.magnitude / Vti.magnitude, 0.0f);//求最大值
		Vni_new = -this.restitution * Vni;
		Vti_new = a * Vti;
		Vi_new = Vni_new + Vti_new;//求出了新的速度
								   //接下来求出冲量
								   //先求矩阵
		Matrix4x4 k = Get_Cross_Matrix(total_positon) * I_inverse * Get_Cross_Matrix(total_positon);
		k = new Matrix4x4(mass_matrix.GetColumn(0) - k.GetColumn(0), mass_matrix.GetColumn(1) - k.GetColumn(1), mass_matrix.GetColumn(2) - k.GetColumn(2), mass_matrix.GetColumn(3) - k.GetColumn(3));
		j = Matrix4x4.Inverse(k).MultiplyVector((Vi_new - Vi));
		Vector3 real_j1 = mass_matrix.MultiplyVector(j);
		Vector3 real_j2 = I_inverse.MultiplyVector((Vector3.Cross(total_positon, j)));
		this.v = this.v + real_j1;
		this.w = w + real_j2;


	}
	bool judgeCollision(Vector3 vertex,Vector3 p,Vector3 N)//判断这个点和平面的关系
    {
		//两个向量构成了一个平面
		//距离函数：
		//（x-p）*n
		float result=Vector3.Dot(vertex-p,N);
		if(result<0.0f)//如果距离函数的值是负的
        {
			return true;
        }
		else
        {
			return false;
        }
    }

	Quaternion add(Quaternion a,Quaternion b)
    {
		return new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);//这里有一个向上的初速度
			w=new Vector3(1,1,0);//先使用一个恒定的角速度来尝试一下
			launched=true;
		}

		// Part I: Update velocities 更新速度、
		//计算重力并且用它来更新速度
		//v-v0=at
		//a=g
		//a=f/m
		//因为最后实际上更新的还是x，v作为中间变量 
		//v=v0-at
		if(launched==true)//如果launche的是true
        {
			//更新线速度
			v = v + grivity * dt;
			v = v * linear_decay;//加上速度衰减
			w=w * angular_decay;//角速度衰减
		}

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));//地面
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x = transform.position;
		x = x + v * dt;//x=x0-vt
					   //Update angular status
					   //这里是根据ppt上的那个公式来写的
		Vector3 dw = 0.5f * dt * w;
		Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
		Quaternion q0 = transform.rotation;
		Quaternion q = add(q0, qw * q0);//根据ppt的公式更新朝向
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
