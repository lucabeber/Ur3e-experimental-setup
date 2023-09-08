# Debian package hosting research

## Initial thought


The initial thought was to create a PPA and provide the debian packages to it.
Relevant links I found from a preliminary research:

- [askubuntu](https://askubuntu.com/questions/71510/how-do-i-create-a-ppa)
- [launchpad.help](https://help.launchpad.net/Packaging/PPA/Uploading)
- [launchpad.help](https://help.launchpad.net/Packaging/PPA)
- [Medium Article](https://medium.com/@labruillere/how-to-create-a-ppa-and-manage-it-like-a-8055-293da8124165)


## Launchpad PPAs are for open-source packages

After creating the process for uploading the packages to launchpad, I stumbled across the following:
- [askubuntu](https://askubuntu.com/questions/234005/is-there-a-repository-ppa-that-collects-closed-source-non-free-software)

So it's not possible to upload closed-source code via PPA (you need to upload source packages, in the form of *.tar.gz - with the source code inside). The packages are built in the Launchpad servers.
 
Looking in the way the ROS folks are doing it, they are using their own servers and you merely import their pgp key in your apt keyring. Then you download their packages (which some can be only binaries).

## List with possible future steps

Then, I searched in google for possible future steps.
Keywords used: `Hosted Package Repository`, `Debian Repository Management`

For use on the cloud, I found the following:
- [Gitlab EE Debian - not yet supported](https://gitlab.com/gitlab-org/gitlab/-/issues/5835)
- [Conan solution with Gitlab - Conan is another package management tool for C/C++, different from apt](https://docs.gitlab.com/ee/user/packages/conan_repository/index.html)
- [Gemfury - private repo is a paid feature](https://gemfury.com/l/debian-repository)
- [Cloudsmith - paid plans only](https://cloudsmith.com/debian-repository)
- [Packagecloud - private repo is a paid feature](https://packagecloud.io)

For use on our own server:
- [Aplty - the Swiss army knife for debian packaging](https://www.aptly.info)
- [fpm](https://fpm.readthedocs.io/en/latest/intro.html)
- [A nice tutorial](https://pmateusz.github.io/linux/2017/06/30/linux-secure-apt-repository.html)
- [Medium article](https://medium.com/sqooba/create-your-own-custom-and-authenticated-apt-repository-1e4a4cf0b864)
- [Debian Official Wiki](https://wiki.debian.org/DebianRepository/Setup)
- [Reprepro](https://www.dynamsoft.com/codepool/linux-debian-reporisory-reprepro.html)
