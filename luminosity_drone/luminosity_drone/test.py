import math
def point_setter(c_sp,next_sp,div):
    difference= list(map(lambda x,y: y -x, c_sp,next_sp))
    diff_norm=list(map(lambda x : x/div,difference))
    # print('Diff',diff_norm)
    new_sps=[]
    new_set_point=c_sp
    for i in range(div):
        new_set_point= list(map(lambda x,y: x + y, new_set_point,diff_norm))
        new_set_point=list(map(lambda x: round(x,2) , new_set_point))
        new_sps.append(new_set_point)
    return new_sps

def array_founder(arr,divisions=10):
    points=[]
    for i in range(len(arr)-1):

        temp_pts=point_setter(arr[i],arr[i+1],divisions)
        for kuku in temp_pts:
            points.append(kuku)
        # print('Final: ',points)
    return points

def land_finder(pt_i,pt_f,dist):
    points=[]
    diff_vect= list(map(lambda x,y: y -x, pt_i,pt_f))
    # print('diff_vect ',diff_vect)
    diff_scalar = math.sqrt(sum([i**2 for i in diff_vect]))
    # print('diff_scalar ',diff_scalar)
    num_pts=int(diff_scalar/dist)
    # print('num pt ',num_pts)
    diff_vect_norm=list(map(lambda x : x/num_pts,diff_vect))
    # print('diff_vect_norm ',diff_vect_norm)
    for land_itr in range(num_pts):
        pt=list(map(lambda x,del_x: x + ((land_itr+1)*del_x), pt_i,diff_vect_norm))
        # print(pt)
        pt=list(map(lambda x: round(x,3) , pt))
        points.append(pt)
    # print(points)
    for i in points:
        i[2] = 25
    return points



c=[0,0,0]
n=[1,2,1]
pts=[[0,0,0],[2,4,6],[5,10,15],[20,15,15]]

print('hi')
# land_finder(c,n,0.1)
print(land_finder(c,n,0.1))
# print(point_setter(c,n,3))
# print(array_founder(pts,5))


