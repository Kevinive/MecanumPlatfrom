%{
һ������O�������ķ��С�������˶��ĳ���,
���룺�����־��룬ǰ���־��룬��ֱ��
��������˶�ѧ���������˶�ѧ����
%}

xd = 225;     %wheel distance in X axis(mm)
yd = 225;     %wheel distance in Y axis(mm)
wheelD = 152.4;     %wheel D(mm)
wheelR = wheelD/2;
ab = (xd + yd)/2;
A = [1,1,-ab;-1,1,-ab;-1,1,ab;1,1,ab];
B = 30/(pi*wheelR);
C = A*B;
D = ((C')*C)\(C');
C
D
disp(['PARAM_A=' num2str(C(1,1))])
disp(['PARAM_B=' num2str(C(3,3))])
disp(['PARAM_C=' num2str(D(1,1))])
disp(['PARAM_D=' num2str(D(3,3))])