# Administração do Wall-e

No cenário deste projeto, administração se refere a manutenção que o administrador pode realizar no contexto de software do Wall-e (Raspberry Pi). Ou seja, reiniciar, desligar, configurar o Wall-e. Além de poder atualizar o código principal do Wall-e para uma versão mais recente remotamente e reiniciá-lo.

A conexão do administrador será feita por SSH, protocolo que visa estabelecer um acesso remoto de forma segura (autenticada e criptografada). Será configurado o Dropbear na Raspberry Pi. Esse é um servidor SSH simples e leve, muito usado em sistemas embarcados. Para se conectar ao servidor na Raspberry Pi, o administrador deve possuir um cliente SSH, por exemplo, o 'dbclient', cliente SSH padrão do Dropbear.

A atualização de código será feita usando a ferramenta de sincronização de arquivos Rsync. Ela é bastante configurável, permitido sincronizar apenas os arquivos selecionados e ignorar os indesejados. Além disso, é possível utilizar o protocolo SSH junto ao Rsync para garantir a segurança e autenticidade dos arquivos sincronizados. Normalmente, seria necessário iniciar um servidor Rsync para que o administrador possa sincronizar os dados com o Wall-e; todavia, operando com SSH, o servidor é iniciado por meio da própria conexão SSH durante a sincronização e finalizado após ela terminar. Assim, não é necessário deixar o servidor Rsync rodando em plano de fundo.


## Chaves e certificados

Assim como para a comunicação usando SSL (mencionado na página ['Comunicação do usuário com o Wall-e'](comunicacao.md)), são necessárias chaves e certificados para estabelecer a comunicação SSH. Na verdade, é possível estabelecer essa comunicação usando a autenticação por senha do usuário, mas é mais seguro usando apenas chaves e certificados.

Quanto a geração das chaves, assim como descrito na página ['Comunicação do usuário com o Wall-e'](comunicacao.md), será escrito um script em Python para gerar as chaves criptográficas usadas pelo administrador.


---
Anterior: [Movimentação do Wall-e no modo autônomo](autonomo.md) | Topo: [Desenvolvimento](README.md) | próximo: [Raspberry Pi](raspberry-pi.md)
