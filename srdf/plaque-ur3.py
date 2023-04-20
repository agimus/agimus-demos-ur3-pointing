handle_fmt="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0 0.01" rpy="0 -1.5707963267948966 0"/>
    <link name="hole_{i:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

print('<robot name="plaque-ur3">')
for k in range(24):
    print(handle_fmt.format(i=k))
print('</robot>')

