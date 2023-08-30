#Begin
begin ="""
<robot name="plaque-tubes">

  <material name="black">
    <color rgba="0 0 0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1.0"/>
  </material>

  <link name="base_link"/>
  
  <!-- Plaque -->
  <link name="plaque_link">
    <visual>
      <geometry>
        <mesh filename="package://agimus_demos/ur3/pointing/meshes/plaque.stl" scale = "0.001 0.001 0.001"/>
      </geometry>
        <material name="white">
          <color rgba="1 1 1 1"/>
        </material>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://agimus_demos/ur3/pointing/meshes/plaque.stl" scale = "0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.2"/>
       <origin xyz="0.129 -0.1535 -0.0025" rpy="0 0 0" />
      <inertia ixx="0.1"  ixy="0"  ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>
  </link>

  <joint name="plaque_joint" type="fixed">
    <parent link="base_link"/>
    <child link="plaque_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
    
   <link name="hole_ref_link">
    <visual>
        <geometry>
            <cylinder radius="0.002" length="0.001" />
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
    </visual>
  </link>
  <joint name="to_hole_ref" type="fixed">
    <parent link="plaque_link" />
    <child link="hole_ref_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
"""

#Nuts
nut="""
 <link name="handle_{i:02}_link">
    <visual>
        <geometry>
            <cylinder radius="0.00135" length="0.007" />
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
    </visual>
  </link>
  <joint name="to_handle_{i:02}" type="fixed">
    <parent link="plaque_link" />
    <child link="handle_{i:02}_link" />
    <origin xyz="{x} {y} 0" rpy="0 0 0" />
  </joint> 
"""

#Screws
screw= """
  <link name="handle_{i:02}_link" />

  <joint name="to_handle_{i:02}" type="fixed">
    <parent link="plaque_link" />
    <child link="handle_{i:02}_link" />
    <origin xyz="{x} {y} 0" rpy="0 0 0" />
  </joint> 
"""

holes="""
  <link name="handle_{i:02}_link" />

  <joint name="to_handle_{i:02}" type="fixed">
    <parent link="plaque_link" />
    <child link="handle_{i:02}_link" />
    <origin xyz="{x} {y} 0" rpy="0 0 0" />
  </joint>
"""
print(begin)
print("<!-- Holes -->")  
print("<!-- 1st line -->") 
I = range(0,12)
X = [0.027, 0.046, 0.0645, 0.083, 0.1015, 0.12, 0.1385, 0.157, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.002, 0.002, 0.002, 0.00225, 0.00225, 0.00225, 0.003, 0.003, 0.003, 0.002, 0.002, 0.002]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.071))
print("<!-- 2nd line -->") 
I = range(12,24) 
X = [0.027, 0.046, 0.0645, 0.083, 0.1015, 0.12, 0.1385, 0.157, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.002, 0.002, 0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.093))    
print("<!-- 3rd line -->")  
I = range(24,34) 
X = [0.027, 0.046, 0.0645, 0.083, 0.1385, 0.157, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.003, 0.003, 0.00225, 0.00225, 0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.115))  
print("<!-- 4th line -->")  
I = range(34, 44)
X = [0.027, 0.046, 0.0645, 0.083, 0.1015, 0.12, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225, 0.00225, 0.00225, 0.003, 0.003]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.192))  
print("<!-- 5th line -->")  
I = range(44, 56)
X = [0.027, 0.046, 0.0645, 0.083, 0.1015, 0.12, 0.1385, 0.157, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225, 0.002, 0.002, 0.00225, 0.00225, 0.002, 0.002]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.214))            
print("<!-- 6th line -->")  
I = range(56, 68)
X = [0.027, 0.046, 0.0645, 0.083, 0.1015, 0.12, 0.1385, 0.157, 0.1755, 0.194, 0.2125, 0.231]
#D = [0.002, 0.002, 0.002, 0.003, 0.003, 0.003, 0.00225, 0.00225, 0.00225, 0.002, 0.002, 0.002]
for i,x in zip(I,X):
    print(holes.format(i=i, x=x, y=-0.236)) 
              
print("<!-- Screws -->")  
print("<!-- 1st line -->")   
I = range(68, 74) 
X = [0.029, 0.05, 0.079, 0.104, 0.139, 0.154] 
for i,x in zip(I,X):
    print(screw.format(i=i, x=x, y=-0.0265)) #d=0.00075
print("<!-- 2nd line -->")   
I = range(74, 80) 
X = [0.029, 0.05, 0.079, 0.104, 0.139, 0.154] 
for i,x in zip(I,X):
    print(screw.format(i=i, x=x, y=-0.049)) #d=0.001
print("<!-- 3rd line -->")   
I = range(80, 83) 
X = [0.1035, 0.129, 0.154]
for i,x in zip(I,X):
    print(screw.format(i=i, x=x, y=-0.258)) #d=0.00125  
I = range(83, 86)     
X = [0.179, 0.204, 0.229]
for i,x in zip(I,X):
    print(screw.format(i=i, x=x, y=-0.257)) #d=0.001  
print("<!-- 4th line -->") 
print(screw.format(i=86, x=0.1035, y=-0.285)) #d=0.00075
print(screw.format(i=87, x=0.129, y=-0.285))  #d=0.00075
print(screw.format(i=88, x=0.154, y=-0.284)) #d=0.00075
print(screw.format(i=89, x=0.179, y=-0.279)) #d=0.00075   
print(screw.format(i=90, x=0.204, y=-0.278)) #d=0.00075
print(screw.format(i=91, x=0.229, y=-0.278)) #d=0.00075

print("<!-- Nuts -->") 
print(nut.format(i=92, x=0.027, y=-0.1535))
print(nut.format(i=93, x=0.083, y=-0.1535))
print(nut.format(i=94, x=0.1755, y=-0.1535))

print('</robot>')