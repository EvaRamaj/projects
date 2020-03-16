"use strict";

import React from 'react';
import ProjectsList from "../components/ProjectsList"
import UserService from "../services/UserService";
import ProjectService from "../services/ProjectService";




export class ProjectsListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            searchterm: "",
            results: []
        };
    }

    componentWillMount(){
        let company = UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined;
        let value = company.id;
        let query = {
            "query" : {
                "term" : {"company.id": value}
            }
        };

        ProjectService.search(query).then((data) => {
            this.setState({
                searchterm: value,
                results: data.hits.hits
            });
        }).catch((e) => {
            console.error(e);
        });

    }



    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <ProjectsList results={this.state.results} error={this.state.error} />

        );
    }
}
