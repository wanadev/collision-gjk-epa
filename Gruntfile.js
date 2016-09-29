module.exports = function(grunt) {

	// Project configuration.
	grunt.initConfig({
		pkg: grunt.file.readJSON('package.json'),

		copy: {
			tests: {
				files: [{
					expand: true,
					cwd: 'tests/',
					src: ['index.html'],
					dest: 'build/tests/'
				}, {
					expand: true,
					cwd: 'tests/',
					src: ['collision-gjk-epa.js'],
					dest: 'build/tests/'
				}, {
					expand: true,
					cwd: 'node_modules/mocha/',
					src: ['mocha.js', 'mocha.css'],
					dest: 'build/tests/'
				}]
			}
		},


		browserify: {
			build: {
				files: {
					'build/<%= pkg.name %>.js': ['collision-gjk-epa.js']
				}
			},
			options: {
				browserifyOptions: {
					debug: true,
					'standalone': 'Collisiongjkepa'
				}
			},
			tests: {
				files: {
					'build/tests/collision-gjk-epa.js': ['tests/collision-gjk-epa.js']
				}
			},
		},

		uglify: {
			build: {
				files: {
					'build/<%= pkg.name %>.min.js': ['build/<%= pkg.name %>.js']
				}
			}
		},

		watch: {
			js: {
				files: ['collision-gjk-epa.js'],
				tasks: ['browserify'],
				options: {
					spawn: false
				}
			}
		},

		clean: {
			dist: ['build'],
			tests: ['build/tests/']
		},

		mocha_phantomjs: {
			all: {
				options: {
					urls: ['http://localhost/collision-gjk-epa/build/tests/']
				}
			}
		}
	});


	grunt.loadNpmTasks('grunt-contrib-clean');
	grunt.loadNpmTasks('grunt-contrib-copy');
	grunt.loadNpmTasks('grunt-contrib-uglify');
	grunt.loadNpmTasks('grunt-browserify');
	grunt.loadNpmTasks('grunt-contrib-watch');
	grunt.loadNpmTasks('grunt-mocha-phantomjs');

	grunt.registerTask('default', ['browserify', 'uglify']);
	grunt.registerTask('test', ['clean:tests', 'copy:tests', 'browserify:tests', 'mocha_phantomjs']);

};
