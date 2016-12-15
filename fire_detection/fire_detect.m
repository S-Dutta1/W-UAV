%from http://rushabhshah10.blogspot.in/2013/10/fire-detecion-with-image-processing.html

i=imread('fire1.JPG');
C = makecform('srgb2Lab');
i_Lab = applycform(i,C);
x=i_Lab(:,:,1);
y=i_Lab(:,:,2);
z=i_Lab(:,:,3);
L=sum(x);
L=sum(L');
a=sum(y);
a=sum(a');
b=sum(z);
b=sum(b');
d=zeros(150,200);
L=L/(150*200);
a=a/(150*200);
b=b/(150*200);
for p=1:1:150
    for q=1:1:200
        if (x(p,q)>L)
            d(p,q)=1;
        else d(p,q)=0;
        end;
    end;
end;
e=zeros(150,200);
for p=1:1:150
    for q=1:1:200
        if (y(p,q)>a)
            e(p,q)=1;
        else e(p,q)=0;
        end;
    end;
end;
f=zeros(150,200);
for p=1:1:150
    for q=1:1:200
        if (z(p,q)>b)
            f(p,q)=1;
        else f(p,q)=0;
        end;
    end;
end;
g=zeros(150,200);
for p=1:1:150
    for q=1:1:200
        if (y(p,q)>z(p,q))
            g(p,q)=1;
        else g(p,q)=0;
        end;
    end;
end;
h=zeros(150,200);
for p=1:1:150
    for q=1:1:200
        if (d(p,q)&&e(p,q)&&f(p,q)&&g(p,q)==1)
            h(p,q)=1;
        else h(p,q)=0;
        end;
    end;
end;
h=sum(h);
h=sum(h');
if (h >10 )
    fprintf('FIRE DETECTED\n');
else
    fprintf('FIRE NOT DETECTED\n');
end;