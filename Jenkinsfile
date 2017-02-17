#!groovy

// Jenkinsfile for compiling, testing, and packaging the Enki libraries.
// Requires CMake plugin from https://github.com/davidjsherman/aseba-jenkins.git in global library.

pipeline {
	agent any // use any available Jenkins agent
	stages {
		stage('Prepare') {
			steps {
				dir('enki') {
					checkout scm
				}
				stash includes: 'enki/**', excludes: '.git', name: 'source'
			}
		}
		stage('Compile') {
			steps {
				parallel (
					"debian" : {
						node('debian') {
							unstash 'source'
							script {
								env.debian_python = sh ( script: '''
									python -c "import sys; print 'lib/python'+str(sys.version_info[0])+'.'+str(sys.version_info[1])+'/dist-packages'"
''', returnStdout: true).trim()
							}
							CMake([sourceDir: pwd()+'/enki', label: 'debian', getCmakeArgs: "-DPYTHON_CUSTOM_TARGET:PATH=${env.debian_python}"])
							stash includes: 'dist/**', name: 'dist-debian'
							stash includes: 'build/**', name: 'build-debian'
						}
					},
					"macos" : {
						node('macos') {
							unstash 'source'
							CMake([sourceDir: pwd()+'/enki', label: 'macos'])
							stash includes: 'dist/**', name: 'dist-macos'
							stash includes: 'build/**', name: 'build-macos'
						}
					},
					"windows" : {
						node('windows') {
							unstash 'source'
							CMake([sourceDir: pwd()+'/enki', label: 'windows'])
							stash includes: 'dist/**', name: 'dist-windows'
							stash includes: 'build/**', name: 'build-windows'
						}
					}
				)
			}
		}
		stage('Test') {
			steps {
				parallel (
					"debian" : {
						node('debian') {
							unstash 'build-debian'
							dir('build/debian') {
								sh 'LANG=C ctest'
							}
						}
					},
					"macos" : {
						node('macos') {
							unstash 'build-macos'
							dir('build/macos') {
								sh 'LANG=C ctest'
							}
						}
					},
					"windows" : {
						node('windows') {
							unstash 'build-windows'
							dir('build/windows') {
								sh 'LANG=C ctest'
							}
						}
					}
				)
			}
		}
		stage('Package') {
			steps {
				parallel (
					"debian" : {
						node('debian') {
							unstash 'dist-debian'
							unstash 'source'
							dir('enki') {
								sh 'which debuild && debuild -i -us -uc -b'
							}
							sh 'mv libenki*.deb libenki*.changes libenki*.build dist/debian/'
							stash includes: 'dist/**', name: 'dist-debian'
						}
					}
				)
			}
		}
		stage('Archive') {
			steps {
				script {
					// Can't use collectEntries yet [JENKINS-26481]
					def p = [:]
					for (x in ['debian','macos','windows']) {
						def label = x
						p[label] = {
							node(label) {
								unstash 'dist-' + label
								archiveArtifacts artifacts: 'dist/**', fingerprint: true, onlyIfSuccessful: true
							}
						}
					}
					parallel p;
				}
			}
		}
	}
}
