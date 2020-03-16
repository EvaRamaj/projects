"use strict";

import React from 'react';

import { ItemDetail } from '../components/ItemDetail';

import MovieService from '../services/MovieService';
import Homepage from "../components/Homepage";
import UserService from "../services/UserService";
import AuthService from "../services/AuthService";


export class HomepageView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props){
        this.setState({
            loading: false,
            str: ''
        });
/*
        let id = this.props.match.params.id;
        console.log(id);
        MovieService.getItem(id).then((data) => {
            this.setState({
                movie: data,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
*/
    }

    redirect(string){
        this.history.push(`/results/${string}`);
    }
    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return ( <Homepage user={this.state.user} onSubmit={this.redirect}/>);

    }
}
