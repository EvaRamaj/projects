"use strict";

import React from 'react';

import UserDetails from '../components/UserDetails';

import ProjectService from '../services/ProjectService';
import ProjectDetails from "../components/ProjectDetails";


export class ProjectDetailView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            project: []
        }
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });
        let id = this.props.match.params.id;
        ProjectService.getProject(id).then((data) => {
            this.setState({
                project: data,
                loading: false
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
            <ProjectDetails project={this.state.project} />
        );
    }
}
