handle_fmt="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0.008 0" rpy="0 1.5707963 -1.5707963"/>
    <link name="hole_{i:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

handle_fmt2="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0.008 0" rpy="0 1.5707963 -1.5707963"/>
    <link name="screw_{j:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

handle_fmt3="""
<handle name="handle_{i:02}" clearance="0.05">
    <position xyz= "0 0 0" rpy="0 1.5707963 -1.5707963"/>
    <link name="nut_{j:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

print('<robot name="plaque-ur3">')
for k in range(24):
    print(handle_fmt.format(i=k))
l = 24    
for k in range(12):
    print(handle_fmt2.format(i=l, j=k))   
    l += 1
for k in range(1,4):
    print(handle_fmt3.format(i=l, j=k))   
    l += 1    
print('</robot>')

"""
g = factory.grippers[0]
for h in factory.handles:
    id = factory.handles.index(h)
    transitionName_21 = '{} < {} | 0-{}_21'.format(g, h, id)
    supervisor.preActions[transitionName_21].preActions = list()
    supervisor.preActions[transitionName_21].preActions.append(wait)
"""