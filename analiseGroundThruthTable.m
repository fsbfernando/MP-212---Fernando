%Analise dos dados pré-treinamento
function analiseGroundThruthTable(tabela_train,tabela_valid,tabela_test)
disp(' ')
disp('Vamos iniciar uma análise rápida do Ground Truth Data ...')


disp('Dados de treinamento ...')
summary( tabela_train)
allBoxes = vertcat( tabela_train.Boxes{:});
aspectRatio = allBoxes(:,3) ./ allBoxes(:,4);
area = prod(allBoxes(:,3:4),2);
h=figure;
subplot(3,1,1)
scatter(area,aspectRatio)
xlabel("Box Area")
ylabel("Aspect Ratio (width/height)");
title("Train Data - Box Area vs. Aspect Ratio")
countEachLabel(boxLabelDatastore(table(tabela_train.Boxes,tabela_train.Labels,'VariableNames',["Boxes","Labels"])))


disp('Dados de validação...')
summary(tabela_valid)
allBoxes = vertcat(tabela_valid.Boxes{:});
aspectRatio = allBoxes(:,3) ./ allBoxes(:,4);
area = prod(allBoxes(:,3:4),2);
figure(h)
subplot(3,1,2)
scatter(area,aspectRatio)
xlabel("Box Area")
ylabel("Aspect Ratio (width/height)");
title("Validation Data - Box Area vs. Aspect Ratio")
countEachLabel(boxLabelDatastore(table(tabela_valid.Boxes,tabela_valid.Labels,'VariableNames',["Boxes","Labels"])))

disp('Dados de teste...')
summary(tabela_test)
allBoxes = vertcat(tabela_test.Boxes{:});
aspectRatio = allBoxes(:,3) ./ allBoxes(:,4);
area = prod(allBoxes(:,3:4),2);
figure(h)
subplot(3,1,3)
scatter(area,aspectRatio)
xlabel("Box Area")
ylabel("Aspect Ratio (width/height)");
title("Test Data - Box Area vs. Aspect Ratio")
countEachLabel(boxLabelDatastore(table(tabela_test.Boxes,tabela_test.Labels,'VariableNames',["Boxes","Labels"])))

end