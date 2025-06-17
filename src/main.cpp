#include "Image.h"          // Inclui a definição da classe Image para manipulação de imagens
#include "ImageSegmenter.h" // Inclui a definição da classe ImageSegmenter
#include <iostream>         // Para entrada e saída de dados (cout, cerr)
#include <string>           // Para manipulação de strings (nomes de arquivos)
#include <set>              // Usado para contar rótulos únicos na depuração do IFT

/**
 * @brief Extrai o nome base de um caminho de arquivo, removendo diretórios e extensão.
 * @param path O caminho completo do arquivo (ex: "src/minha_imagem.png").
 * @return O nome base do arquivo (ex: "minha_imagem").
 */
std::string getBaseName(const std::string& path) {
    // Encontra a última ocorrência de '/' ou '\' para o separador de diretório
    size_t slash = path.find_last_of("/\\");
    // Encontra a última ocorrência de '.' para a extensão do arquivo
    size_t dot = path.find_last_of(".");
    
    // Se não houver ponto (sem extensão), considera que o nome vai até o final da string
    if (dot == std::string::npos) dot = path.length();
    
    // Retorna a substring que corresponde ao nome base
    // Começa após o último separador de diretório (ou do início, se não houver)
    // e termina antes do ponto da extensão (ou do final da string)
    return path.substr(slash + 1, dot - slash - 1);
}

/**
 * @brief Função principal do programa.
 * Lida com o carregamento da imagem, execução dos algoritmos de segmentação
 * (Felzenszwalb e IFT) e salvamento dos resultados.
 * @return 0 se a execução for bem-sucedida, 1 em caso de erro.
 */
int main() {
    // *** DEFINIÇÃO DA IMAGEM DE TESTE ***
    // Altere este caminho para a imagem que deseja testar.
    // Certifique-se de que a imagem (ex: lego.png, coffe-table.png, lenna-RGB.png, lennaGray.png)
    // está localizada no diretório 'src/'.
    std::string inputFilename = "src/lego.png"; // Imagem de entrada atual

    // Cria um objeto Image carregando a imagem do caminho especificado.
    Image img(inputFilename);

    // Verificação básica para garantir que a imagem foi carregada corretamente.
    if (img.width == 0 || img.height == 0) {
        std::cerr << "ERRO: Não foi possível carregar a imagem: " << inputFilename << ". Verifique o caminho ou se ela existe." << std::endl;
        return 1; // Sai com código de erro se a imagem não puder ser carregada.
    }

    // Cria um objeto ImageSegmenter associado à imagem carregada.
    ImageSegmenter segmenter(img);

    // Obtém o nome base do arquivo (sem o caminho ou extensão) para nomear os arquivos de saída.
    std::string baseName = getBaseName(inputFilename);

    // --- EXECUÇÃO E SALVAMENTO DA SEGMENTAÇÃO FELZENSZWALB ---
    // O parâmetro 'k' controla a granularidade da segmentação Felzenszwalb.
    // - Valores maiores de 'k' resultam em menos segmentos (maiores e mais homogêneos).
    // - Valores menores de 'k' resultam em mais segmentos (menores e mais detalhados).
    // O valor ideal de 'k' é experimental e depende da imagem de entrada.
    auto labels_fz = segmenter.segmentGraphFelzenszwalb(250.0); // Valor de 'k' ajustado para lego.png
    auto out_fz = segmenter.visualizeSegmentation(labels_fz); // Visualiza os rótulos em cores aleatórias
    out_fz.save(baseName + "_felzenszwalb.png"); // Salva a imagem segmentada
    std::cout << "Segmentação Felzenszwalb gerada: " << baseName << "_felzenszwalb.png" << std::endl;


    // --- EXECUÇÃO E SALVAMENTO DA SEGMENTAÇÃO IFT ---
    // As 'seeds' (sementes) são pontos iniciais a partir dos quais o IFT expande as regiões.
    // Seu posicionamento é crucial para o resultado do IFT.
    // As coordenadas (linha, coluna) são convertidas para um índice linear usando img.index().
    // Estas sementes são escolhidas especificamente para a imagem 'lego.png' para cobrir
    // diferentes objetos (caixa, blocos coloridos, fundo).
    std::vector<int> seeds = {
        img.index(100, 250),   // Topo da caixa (exemplo)
        img.index(300, 400),   // Lado da caixa (exemplo)
        img.index(400, 100),   // Bloco azul (exemplo)
        img.index(400, 200),   // Bloco vermelho (exemplo)
        img.index(400, 300),   // Bloco amarelo (exemplo)
        img.index(50, 450),    // Fundo branco (exemplo)
        img.index(450, 450)    // Fundo branco (exemplo)
    };

    // --- REFERÊNCIAS DE SEMENTES PARA OUTRAS IMAGENS (COMENTADAS) ---
    // Mantenha estas linhas comentadas; elas servem apenas como exemplo e referência
    // para outras imagens que você possa ter testado ou queira testar no futuro.
    /*
    std::vector<int> coffe_table_seeds = {
        img.index(100, 350),   // Xícara de cima (com café)
        img.index(300, 300),   // Xícara de baixo (vazia)
        img.index(150, 150),   // Óculos
        img.index(250, 100),   // Livro
        img.index(400, 400),   // Mesa (fundo)
        img.index(50, 50)      // Mesa (fundo superior esquerdo)
    };
    */
    /*
    std::vector<int> lenna_seeds = {
        img.index(50, 250),
        img.index(150, 250),
        img.index(300, 150),
        img.index(300, 400),
        img.index(450, 250)
    };
    */

    // Verificação de segurança para garantir que todas as sementes estão dentro dos limites da imagem.
    for (size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i] < 0 || seeds[i] >= img.width * img.height) {
            std::cerr << "ERRO: Semente " << i << " fora dos limites da imagem: " << seeds[i] << std::endl;
            return 1; // Sai com código de erro se uma semente for inválida.
        }
    }

    // Executa o algoritmo IFT com as sementes definidas.
    auto labels_ift = segmenter.segmentGraphIFT(seeds);
    auto out_ift = segmenter.visualizeSegmentation(labels_ift); // Visualiza os rótulos
    out_ift.save(baseName + "_ift.png"); // Salva a imagem segmentada pelo IFT
    std::cout << "Segmentação IFT gerada: " << baseName << "_ift.png" << std::endl;

    // --- CÓDIGO DE DEPURACÃO PARA O IFT ---
    // Este bloco calcula e imprime o número de rótulos únicos gerados pelo IFT.
    // Isso ajuda a entender a granularidade da segmentação do IFT.
    std::set<int> unique_ift_labels;
    for (int label : labels_ift) {
        unique_ift_labels.insert(label);
    }
    std::cout << "\n--- DEPURACAO IFT ---" << std::endl;
    std::cout << "Número de sementes fornecidas: " << seeds.size() << std::endl;
    std::cout << "Número de rótulos únicos na saída do IFT: " << unique_ift_labels.size() << std::endl;
    if (unique_ift_labels.size() == 1) {
        std::cout << "ATENÇÃO: O IFT resultou em uma única região. Isso pode indicar que as sementes não foram eficazes ou que a função de custo é muito permissiva." << std::endl;
    } else if (unique_ift_labels.size() < seeds.size()) {
        std::cout << "AVISO: O número de rótulos únicos (" << unique_ift_labels.size()
                  << ") é menor do que o número de sementes fornecidas (" << seeds.size() << ")." << std::endl;
        std::cout << "Isso pode ocorrer se múltiplas sementes convergirem para a mesma região ou se algumas sementes foram 'engolidas' por outras." << std::endl;
    }
    std::cout << "--- FIM DEPURACAO IFT ---\n" << std::endl;

    std::cout << "Processo de segmentação concluído!\n";
    return 0; // Retorna 0 indicando sucesso.
}