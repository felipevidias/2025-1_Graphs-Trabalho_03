# Projeto de Segmentação de Imagens (Felzenszwalb e IFT)

Este projeto implementa dois algoritmos clássicos de segmentação de imagens: o algoritmo de Segmentação por Componentes Conectados de Felzenszwalb e a Transformada da Floresta de Imagem (IFT - Image Forest Transform).

## Visão Geral

O objetivo principal deste projeto é demonstrar e comparar a aplicação de dois métodos distintos para segmentar imagens digitais.

* **Segmentação Felzenszwalb (Graph-Based Image Segmentation):** É um algoritmo não-supervisionado que agrupa pixels em regiões (segmentos) baseando-se em um critério que compara a diferença de intensidade/cor entre pixels dentro de uma região com a diferença entre pixels em diferentes regiões. Ele é eficaz para capturar estruturas visuais em diversas escalas.

* **Transformada da Floresta de Imagem (IFT):** É um algoritmo supervisionado (ou semi-supervisionado) que realiza uma propagação de rótulos a partir de "sementes" (pixels iniciais) em um grafo, onde cada pixel é atribuído à semente para a qual o "custo" do caminho é mínimo. É versátil e pode ser usado para segmentação, classificação, entre outras tarefas.

O projeto permite carregar imagens (coloridas ou em tons de cinza), aplicar os algoritmos de segmentação e salvar as imagens segmentadas com cores aleatórias para visualização.

## Estrutura do Projeto

* `src/`: Contém os arquivos de código-fonte (`.cpp`).
    * `main.cpp`: Ponto de entrada do programa, lida com o carregamento da imagem, execução dos algoritmos e salvamento dos resultados.
    * `Image.cpp`: Implementa a classe `Image` para carregar, manipular pixels e salvar imagens.
    * `ImageSegmenter.cpp`: Contém as implementações dos algoritmos de segmentação Felzenszwalb e IFT, além de funções auxiliares como `pixelDifference` e `visualizeSegmentation`.
    * `DSU.cpp`: Implementa a estrutura de dados Disjoint Set Union (DSU), utilizada pelo algoritmo de Felzenszwalb.
* `include/`: Contém os arquivos de cabeçalho (`.h`) para as classes e estruturas.
* `lib/`: Contém bibliotecas de terceiros necessárias (e.g., `stb_image.h`, `stb_image_write.h` para carregamento/salvamento de imagens).
* `src/` (imagens de teste): Coloque suas imagens `.png` ou `.jpg` de teste aqui. Exemplos: `lenna-RGB.png`, `lennaGray.png`, `coffe-table.png`, `lego.png`.

## Como Compilar e Executar

Para compilar e executar o projeto, você precisará de um compilador C++ (como o G++).

1.  **Navegue até a pasta raiz do projeto** no seu terminal.
2.  **Compile o projeto** usando o seguinte comando:

    ```bash
    g++ src/*.cpp -Iinclude -Ilib -o segmentador
    ```

    * `g++ src/*.cpp`: Compila todos os arquivos `.cpp` no diretório `src/`.
    * `-Iinclude`: Inclui o diretório `include/` para os arquivos de cabeçalho (`.h`).
    * `-Ilib`: Inclui o diretório `lib/` para as bibliotecas de terceiros.
    * `-o segmentador`: Gera um executável chamado `segmentador`.

3.  **Execute o programa** usando o comando:

    ```bash
    ./segmentador
    ```

    O programa processará a imagem configurada em `main.cpp` e salvará os resultados de segmentação (por exemplo, `coffe-table_felzenszwalb.png` e `coffe-table_ift.png`) no mesmo diretório da imagem de entrada (`src/`).

## Imagens de Teste

O `main.cpp` está configurado para testar com uma imagem específica (atualmente `src/lego.png`). Para testar com outras imagens:

1.  Coloque suas imagens de teste (e suas versões em tons de cinza, se aplicável) no diretório `src/`.
2.  Edite o arquivo `main.cpp` e altere a variável `inputFilename` para o nome do seu novo arquivo de imagem (ex: `std::string inputFilename = "src/sua_nova_imagem.png";`).
3.  Ajuste as coordenadas das `seeds` para o algoritmo IFT, pois elas são específicas para cada imagem. As sementes comentadas no `main.cpp` servem como exemplo para outras imagens.
4.  Ajuste o parâmetro `k` para o Felzenszwalb e o `AMPLIFICATION_FACTOR` em `ImageSegmenter.cpp` para o IFT. Esses valores são críticos para o desempenho e a granularidade da segmentação em diferentes imagens.

## Observações sobre os Resultados

* **Felzenszwalb:** Geralmente produz bons resultados visuais em imagens naturais, criando segmentos coerentes. A qualidade pode ser ajustada pelo parâmetro `k`.
* **IFT:** O IFT (com sementes manuais esparsas e `pixelDifference` baseada em distância de cor) pode ter dificuldade em segmentar imagens naturais complexas (como a Lenna) em muitos objetos distintos. Ele tende a criar poucas e grandes regiões, mesmo com a amplificação dos pesos das arestas. Para obter resultados mais detalhados com o IFT em tais imagens, seriam necessárias estratégias de semente mais avançadas (e.g., sementes automáticas baseadas em mínimos de gradiente) ou funções de custo mais sofisticadas. Para imagens com bordas muito nítidas e cores sólidas (como `lego.png`), o IFT pode apresentar um desempenho mais visível e criar mais segmentos.

Sinta-se à vontade para experimentar com diferentes imagens e ajustar os parâmetros (`k` e `AMPLIFICATION_FACTOR`) para observar o comportamento de cada algoritmo.
