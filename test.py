line = [(-0.2867132867132867, 375.1958041958042), (-0.013986013986013986, 204.23776223776224),(-0.013886013986013986, 204.23776223776224),
        (-0.2857142857142857, 382.85714285714283), (-0.03292181069958848, 216.3127572016461)]

result_list=[]
for ii in xrange(len(line)):
    start=line[ii][0]
    for i in line:
        print i[0]
        if abs(abs(i[0])-abs(start))<=0.01 and abs(abs(i[0])-abs(start))!=0:
            result_list.append(i)
        else:
            pass
slop_sum=0
slop_sum_final=0
intercept_sum_final=0
for i in result_list:
    slop_sum+=i[0]
avg_slope=slop_sum/len(result_list)
result_list_final=[]
for i in result_list:
    if abs(i[0])>abs(avg_slope):
        result_list_final.append(i)
    else:
        pass
for i in result_list_final:
    slop_sum_final+=i[0]
    intercept_sum_final+=i[1]
print result_list
# print "result",(slop_sum/len(result_list),intercept_sum/len(result_list))
print result_list_final
print "result",(slop_sum_final/len(result_list_final),intercept_sum_final/len(result_list_final))