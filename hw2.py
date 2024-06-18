

with open('C:/Users/송재준/Desktop/python파일/list1.txt', 'r') as 파일:
    메모장1 = (파일.read()).split('\n')
with open('C:/Users/송재준/Desktop/python파일/1.txt', 'r') as 파일:
    메모장2 = (파일.read()).split('\n')


def 점수계산기(a):
    문자목록 = []
    for i in range(len(a)):
        if i == 0:
            문자목록.append(1)
        elif a[i-1] == a[i]:
            문자목록[len(문자목록)-1] +=1
        else :
            문자목록.append(1)
    합계 = 0
    for i in 문자목록:
        합계 += i * i
    
    점수 = 합계 / len(a)
    return 점수



점수모음2 = []

for i in range(len(메모장2)):
        점수모음2.append(점수계산기(메모장2[i]))

print(메모장2[점수모음2.index(max(점수모음2))])

점수모음1 = []

for i in range(len(메모장1)):
        점수모음1.append(점수계산기(메모장1[i]))

print(메모장1[점수모음1.index(max(점수모음1))])


메모장3 = []
for i in 메모장2:
    for k in 메모장1:
        메모장3.append(i+k)

점수모음3 = []

for i in range(len(메모장3)):
        점수모음3.append(점수계산기(메모장3[i]))

print(메모장3[점수모음3.index(max(점수모음3))])
print(max(점수모음3))