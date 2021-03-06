template<>
struct VLP16_WebServer_Template < Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS >
        : public VLP16_WebServer {
    VLP16_WebServer_Template() {}
    std::string request (const WebServerCommands    & _cmd)         const override;
    int         send    (const VLP16_settingsConfig & _config)      const override;
};

template<>
struct VLP16_WebServer_Template < Velodyne_WebServer::WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS >
        : public VLP16_WebServer {
    std::string request (const WebServerCommands    & _cmd)         const override;
    int         send    (const VLP16_settingsConfig & _config)      const override;
};

template<>
struct VLP16_WebServer_Template < Velodyne_WebServer::WebServerConnectionType::CURL >
        : public VLP16_WebServer {
    std::string request (const WebServerCommands    & _cmd)         const override;
    int         send    (const VLP16_settingsConfig & _config)      const override;
};
