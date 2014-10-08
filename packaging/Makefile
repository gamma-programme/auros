all: repository

clean:
	rm -rf db pool public staging *.changes *.dsc *.tar.gz

repository:
	rm -rf staging db pool public && mkdir staging
	cp incoming/*.deb staging/
	
	equivs-build --full auros-developer.equiv
	equivs-build --full auros-aircraft.equiv
	rm *.changes *.dsc *.tar.gz
	mv *.deb staging
	
	aptly repo create auros
	aptly repo add auros ./staging/*.deb
	aptly publish -distribution="trusty" -architectures=i386,amd64 repo auros
