handle_fmt="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0.008 0" rpy="0 1.5707963 -1.5707963"/>
    <link name="handle_{i:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

handle_fmt2="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0 0" rpy="0 1.5707963 -1.5707963"/>
    <link name="handle_{i:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

print('<robot name="plaque-ur3">')
for i in range(0,92):
    print(handle_fmt.format(i=i))

for i in range(92, 95):
    print(handle_fmt2.format(i=i))
print('</robot>')
