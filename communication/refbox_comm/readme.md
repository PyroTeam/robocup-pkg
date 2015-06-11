Communication avec la Referee Box
=================================

Ce package est actuellement fonctionnel avec la branche de développement **timn/rcll-2015** de la refereeBox

Description
-----------
Ce package implémente la communication avec la referee Box en convertissant les messages au format ROS utilisable par les autres noeuds.

Noeuds
------
* `refBoxComm` : noeud principal de la communication

Launchfiles
-----------
* `refBoxComm.launch` : lance le noeud `refBoxComm` et donne la possibilité de modifier divers paramètres au lancement (nom d'équipe, couleur d'équipe, numéro et nom du robot)
* `refDebugddd.launch` : idem que `refBoxComm.launch`, mais en lançant le noeud en mode debug via le logiciel ddd

Installation de la referee Box
------------------------------

La procédure d'installation est décrite sur le wiki de la refereeBox dans la section reservée à Ubuntu 14.04, [ici](https://trac.fawkesrobotics.org/wiki/LLSFRefBox/Install#InstallRefboxonUbuntu14.04trusty).
J'indique ici la procédure complète car plusieurs modifications sont necessaires pour que le package refbox_comm compile correctement :

Installer les dépendances :

    sudo apt-get install libmodbus-dev protobuf-compiler libprotobuf-dev libprotoc-dev libboost-all-dev \
                         libglibmm-2.4-dev libgtkmm-3.0-dev libncursesw5-dev libyaml-cpp-dev libavahi-client-dev git \
                         libxt-dev libxaw7-dev libncurses5-dev autoconf autogen libtool libyaml-dev

Installation de Clips 6.30 : ​http://sourceforge.net/projects/clipsmm/files/clips/6.30.0.20090722svn/clips-6.30.0.20090722svn.tar.bz2/download

Compiler :

    ./configure && make

Installer :

    sudo make install

Installation de Clipsmm (c++ clips interface) : ​http://sourceforge.net/projects/clipsmm/files/clipsmm/clipsmm-0.3.4.tar.bz2/download

Compiler:

    ./autogen.sh && ./configure && make

Installer

    sudo make install


Créer un dossier refbox dans votre dossier personnel

    cd ~ && mkdir refbox && cd refbox

Installer la refBox (branche timn/rcll-2015) :

    git clone http://git.fawkesrobotics.org/llsf-refbox.git
    cd llsf-refbox
    git checkout timn/rcll-2015
    make
