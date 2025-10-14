%% Forma Tabela de Dados Preliminar para o DataStore

%Deve rodar a função readDadosFilePytorch antes
%Parâmetros de entrada
%          1. classes = tabela lida do arquivo .txt da RoboFlow com os
%          nomes das classes
%          2. annotations = tabela lida do arquivo .txt da RoboFlow com os
%          dados de anotação
%          2. nomedaPasta = 'CableShark_MultiClass.v1i.yolov4pytorch_Multiclass'; %Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
%          3. op = 'train' ou 'valid' ou 'test'

function tabela = formaTable(classes,annotations,nomedaPasta,op)

NumClasses = length(classes.Classes);
NumImagens = size(annotations.imageFileName,1);
NumCampos = size(annotations,2);
iv=1;

for(i=1:NumImagens)  %Percorre os dados de acordo com o número de imagens
    j=2;
    ok = true;
    bb_aux=[];
    ll_aux=[];
    while(j<=NumCampos)
        if(j==2 && isnan(annotations.(strcat('VarName',num2str(j)))(i)))
            ok = false; %Se o primeiro campo VarName2 é Nan, já descarta a imagem atual
            break;
        end
        b_x_corner = annotations.(strcat('VarName',num2str(j)))(i)+1;
        b_y_corner= annotations.(strcat('VarName',num2str(j+1)))(i)+1;
        b_width = annotations.(strcat('VarName',num2str(j+2)))(i)-annotations.(strcat('VarName',num2str(j)))(i);
        b_length = annotations.(strcat('VarName',num2str(j+3)))(i)-annotations.(strcat('VarName',num2str(j+1)))(i);
        for(t=0:NumClasses-1)
            if(annotations.(strcat('VarName',num2str(j+4)))(i)==t)
                nomeClass=classes.Classes(t+1);
                break;
            end
        end
        if(isnan(b_x_corner)||isnan(b_y_corner)||isnan(b_width)||isnan(b_length))
            break;
        else
            if(isempty(bb_aux))
                bb_aux=[b_x_corner,b_y_corner,b_width,b_length];
                ll_aux=nomeClass;
            else
                bb_aux=[bb_aux;b_x_corner,b_y_corner,b_width,b_length];
                ll_aux=[ll_aux;nomeClass];
            end
        end
        j=j+5;
    end
    if(ok)
        IM_Cell{iv,1}=char(strcat(nomedaPasta,'/',op,'/',annotations.imageFileName(i,:))); %Armazena qual é a imagem
        BB_Cell{iv,1}=bb_aux;%Armazena um cell array com todos os BB da i-th imagem
        LL_Cell{iv,1}=ll_aux;%Armazena um cell array com todos os labels/classes da i-th imagem
        iv=iv+1;
    end
end

%!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
tabela= table(IM_Cell,BB_Cell,LL_Cell,'VariableNames',["imageFilename","Boxes","Labels"]);
tabela.imageFilename= fullfile(pwd,string(tabela.imageFilename)); %caminho completo ate chegar nas imagens

end